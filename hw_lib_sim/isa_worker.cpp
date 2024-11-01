/*
 * SPDX-FileCopyrightText: Korneliusz Osmenda <korneliuszo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include <array>
#include <cstring>
#include <cassert>

#include <isa_worker.hpp>

#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>
#include <cstdio>
#include <atomic>

volatile bool dma_single_transfer;

void ISA_Pre_Init()
{
	// ************ Pico I/O Pin Initialisation ****************
	// * Put the I/O in a correct state to avoid PC Crash      *


}

volatile bool TC_triggered_val;

volatile uint8_t * volatile rx_ptr;
volatile size_t rx_len = 0;
volatile bool dma_sent = false;
const volatile uint8_t * volatile tx_ptr;
volatile size_t tx_len = 0;

static void try_rx_dma(int accept_socket_fd)
{
	if(!dma_sent && rx_len)
	{
		uint8_t sbuff[2];
		sbuff[0]=0x0B;
		sbuff[1]=1;
		send(accept_socket_fd,sbuff,2,0);
		dma_sent=true;
	}
}

static void try_tx_dma(int accept_socket_fd)
{
	if(!dma_sent && tx_len)
	{
		uint8_t sbuff[4];
		sbuff[0]=0x0D;
		sbuff[1]=1;
		sbuff[2]=0;
		sbuff[3] = *tx_ptr;
		send(accept_socket_fd,sbuff,4,0);
		dma_sent=true;
	}
}

static bool dma_resp(int accept_socket_fd, uint8_t *buff,size_t len)
{
	if(buff[0] == 0x0C)
	{
		dma_sent = false;
		if (buff[1] & 0x01)
		{
			try_rx_dma(accept_socket_fd);
			return true;
		}
		if(buff[1] & 0x02)
			TC_triggered_val = true;
		if(rx_len)
		{
			*rx_ptr++ = buff[3];
			rx_len--;
		}
		try_rx_dma(accept_socket_fd);
		return true;
	}
	if(buff[0] == 0x0E)
	{
		dma_sent = false;
		if (buff[1] & 0x01)
		{
			try_tx_dma(accept_socket_fd);
			return true;
		}
		if(buff[1] & 0x02)
			TC_triggered_val = true;
		if(tx_len)
		{
			tx_ptr++;
			tx_len--;
		}
		try_tx_dma(accept_socket_fd);
		return true;
	}
	if(buff[0] == 0x10)
	{
		dma_sent = false;
		return true;
	}
	if(buff[0] == 0x12)
	{
		dma_sent = false;
		return true;
	}
	return false;
}

volatile int curr_conn;

void SetupSingleTransferTXDMA(uint dma_chan, const volatile uint8_t * buff, size_t len)
{
	TC_triggered_val = false;
    dma_single_transfer = true;
	tx_ptr = buff;
	tx_len = len;
	//try_tx_dma(curr_conn);
}

void SetupSingleTransferRXDMA(uint dma_chan, volatile uint8_t * buff, size_t len)
{
	TC_triggered_val = false;
    dma_single_transfer = true;
	rx_ptr = buff;
	rx_len = len;
}

bool DMA_Complete(uint dma_chan)
{
	return !tx_len && !rx_len;
}

bool TC_Triggered()
{
	if(TC_triggered_val)
	{
		TC_triggered_val = false;
		return true;
	}
	return false;
}

void ISA_Init()
{

}

struct Device_int
{
	uint32_t offset = 0;
	uint32_t size = 0;
	uint32_t (*rdfn)(void*, uint32_t);
	void (*wrfn)(void*, uint32_t,uint8_t);
	void * obj;
};


constexpr uint32_t MEM_RD = (1<<(20+8+3-8));
constexpr uint32_t MEM_WR = (1<<(20+8+2-8));
constexpr uint32_t IO_RD = (1<<(20+8+1-8));
constexpr uint32_t IO_WR = (1<<(20+8+0-8));


std::array<Device_int,4> devices_mem = {};
std::array<Device_int,4> devices_io = {};

size_t used_devices_mem = 0;
size_t used_devices_io = 0;

bool add_device(const Device & device)
{
	switch(device.type)
	{
	case Device::Type::MEM:
		if(used_devices_mem == devices_mem.size())
			return false;
		{
			devices_mem[used_devices_mem] =
			{
					.offset = device.start,
					.size = device.size,
					.rdfn = device.rdfn,
					.wrfn = device.wrfn,
					.obj = device.obj,
			};

		}
		used_devices_mem++;
		break;
	case Device::Type::IO:
		if(used_devices_io == devices_io.size())
			return false;
		{
			devices_io[used_devices_io] =
			{
					.offset = device.start,
					.size = device.size,
					.rdfn = device.rdfn,
					.wrfn = device.wrfn,
					.obj = device.obj,
			};
		}
		used_devices_io++;

		break;
	default:
		return false;
	}
	return true;
}

std::atomic<uint16_t> irqs;
uint16_t irqs_state;

static void try_update_irq(int accept_socket_fd)
{
	uint16_t irqs_cache = irqs;
	if(!dma_sent && irqs_cache !=irqs_state)
	{
		uint16_t irqs_changed = irqs_cache^irqs_state;
		for(int i=0;i<16;i++)
			if((1UL<<i)&irqs_changed)
			{
				uint8_t sbuff[2];
				sbuff[0]=((1<<i)&irqs_cache) ? 0x0F : 0x11;
				sbuff[1]=i;
				send(accept_socket_fd,sbuff,2,0);
				dma_sent=true;
				irqs_state^=(1<<i);
				break;
			}
	}
}

static void* ISA_comm(void* arg)
{
    struct sockaddr_un sockaddr_un = {0};

	int sock = socket(AF_UNIX,SOCK_SEQPACKET,0);
	assert(sock !=-1);

    /* Construct the bind address structure. */
    sockaddr_un.sun_family = AF_UNIX;
    strcpy( sockaddr_un.sun_path, "/tmp/86extsock" );

    int return_value =
        bind(
        	sock,
            (struct sockaddr *) &sockaddr_un,
            sizeof( struct sockaddr_un ) );

    /* If socket_address exists on the filesystem, then bind will fail. */
    if ( return_value == -1 ) assert( 0 );

    if ( listen( sock, 1 ) == -1 ) assert( 0 );

    while(1)
    {
    	int accept_socket_fd;
        accept_socket_fd = accept( sock, NULL, NULL );
        if ( accept_socket_fd == -1 ) assert( 0 );

        uint8_t buff[9];
        for(int i=0;i<used_devices_mem;i++)
        {
        	buff[0]=1;
        	buff[1]=devices_mem[i].offset>>24;
        	buff[2]=devices_mem[i].offset>>16;
        	buff[3]=devices_mem[i].offset>>8;
        	buff[4]=devices_mem[i].offset>>0;
        	buff[5]=devices_mem[i].size>>24;
        	buff[6]=devices_mem[i].size>>16;
        	buff[7]=devices_mem[i].size>>8;
        	buff[8]=devices_mem[i].size>>0;
            send(accept_socket_fd,buff,9,0);
        }
        for(int i=0;i<used_devices_io;i++)
        {
        	buff[0]=6;
        	buff[1]=devices_io[i].offset>>8;
        	buff[2]=devices_io[i].offset>>0;
        	buff[3]=devices_io[i].size>>8;
        	buff[4]=devices_io[i].size>>0;
            send(accept_socket_fd,buff,5,0);
        }
        buff[0] = 0;
        send(accept_socket_fd,buff,1,0);
        {
        	  struct timeval timeout;
        	    timeout.tv_sec = 0;
        	    timeout.tv_usec = 1000;
        	    assert(setsockopt (accept_socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,sizeof timeout) >= 0);
        }
        curr_conn = accept_socket_fd;
        while(1)
        {
    		try_update_irq(accept_socket_fd);
        	try_rx_dma(accept_socket_fd);
    		try_tx_dma(accept_socket_fd);
        	uint8_t rbuff[32];
        	int rlen = recv(accept_socket_fd,rbuff,32,0);
        	if((rlen == -1) && ((errno == EAGAIN) || (errno == EWOULDBLOCK)))
        		continue;
        	if (rlen <0)
        	{
        		break;
        	}
        	if(dma_resp(accept_socket_fd,rbuff,rlen))
        		continue;
        	if (rbuff[0]==0x02)
        	{
        		uint32_t addr = (rbuff[1]<<24)|(rbuff[2]<<16)|(rbuff[3]<<8)|(rbuff[4]<<0);
        		uint8_t readout = 0xff;
                for(int i=0;i<used_devices_mem;i++)
                {

                	if(addr>=devices_mem[i].offset &&
                		addr<devices_mem[i].offset+devices_mem[i].size)
                	{
                		readout = devices_mem[i].rdfn(devices_mem[i].obj,addr-devices_mem[i].offset);
                		break;
                	}
                }
                uint8_t sbuff[2];
                sbuff[0] = 0x03;
                sbuff[1] = readout;
                send(accept_socket_fd,sbuff,2,0);
        	}
        	else if (rbuff[0]==0x04)
        	{
        		uint32_t addr = (rbuff[1]<<24)|(rbuff[2]<<16)|(rbuff[3]<<8)|(rbuff[4]<<0);
                for(int i=0;i<used_devices_mem;i++)
                {

                	if(addr>=devices_mem[i].offset &&
                		addr<devices_mem[i].offset+devices_mem[i].size)
                	{
                		devices_mem[i].wrfn(devices_mem[i].obj,addr-devices_mem[i].offset,rbuff[5]);
                		break;
                	}
                }
                uint8_t sbuff[2];
                sbuff[0] = 0x05;
                send(accept_socket_fd,sbuff,1,0);
        	}
        	else if (rbuff[0]==0x07)
        	{
        		uint32_t addr = (rbuff[1]<<8)|(rbuff[2]<<0);
        		uint8_t readout = 0xff;
                for(int i=0;i<used_devices_io;i++)
                {

                	if(addr>=devices_io[i].offset &&
                		addr<devices_io[i].offset+devices_io[i].size)
                	{
                		readout = devices_io[i].rdfn(devices_io[i].obj,addr-devices_io[i].offset);
                		break;
                	}
                }
                uint8_t sbuff[2];
                sbuff[0] = 0x08;
                sbuff[1] = readout;
                send(accept_socket_fd,sbuff,2,0);
        	}
        	else if (rbuff[0]==0x09)
        	{
        		uint32_t addr = (rbuff[1]<<8)|(rbuff[2]<<0);
                for(int i=0;i<used_devices_io;i++)
                {

                	if(addr>=devices_io[i].offset &&
                		addr<devices_io[i].offset+devices_io[i].size)
                	{
                		devices_io[i].wrfn(devices_io[i].obj,addr-devices_io[i].offset,rbuff[3]);
                		break;
                	}
                }
                uint8_t sbuff[2];
                sbuff[0] = 0x0A;
                send(accept_socket_fd,sbuff,1,0);
        	}
        	else
        	{
        		assert(0);
        	}
        }

        close(accept_socket_fd);
    }
}


void ISA_Start()
{
    pthread_t ptid;

    // Creating a new thread
    pthread_create(&ptid, NULL, &ISA_comm, NULL);
}

struct IRQh {
	bool lit;
	uint8_t address;
};

static constexpr size_t IRQh_len = 10;

static IRQh irqhandlers[IRQh_len];
static size_t IRQh_used;

void IRQ_Handle_Change_IRQ(uint8_t irqh,uint8_t irq)
{
	irqhandlers[irqh].address=irq;
}


uint8_t IRQ_Create_Handle(uint8_t irq)
{
	if(IRQh_used >= IRQh_len)
		return 0xff;
	IRQ_Handle_Change_IRQ(IRQh_used,irq);
	return IRQh_used++;
}

void IRQ_Set(uint8_t irqh, bool val)
{
	if(irqh >= IRQh_used)
		return;
	irqhandlers[irqh].lit = val;
	uint8_t orirq = 0;
	for(size_t i=0;i<IRQh_used;i++) // first handler prioritized
	{
		if(irqhandlers[i].lit)
		{
			orirq = irqhandlers[i].address;
			break;
		}
	}
	if(!orirq)
		irqs = 0;
	else
		irqs = 1<<orirq;
}

