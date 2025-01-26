#pragma once
#include <stddef.h>
#include <type_traits>
#include <string.h>

template <typename T, int SIZE>
class Fifo
{
protected:
	volatile T data[SIZE];

	using Tp =  volatile T *;

	struct MemoryRegion {
		const Tp data_start;
		const Tp data_end;
	};

	const MemoryRegion mr = {data, &data[SIZE]};

	struct Iterator {
		struct Dereference {
			Tp start;
			Tp stop;
			Tp start2;
			Tp stop2;
			Dereference(const Iterator it)
			: start(it.ptr)
			, stop(it.mr.data_end)
			, start2(it.mr.data_start)
			, stop2(it.ptr)
			{};
			void set_len(size_t len)
			{
				std::make_signed<size_t>::type slen = len - (stop-start);
				if (slen < 0)
				{
					stop2=start2;
					stop = stop + slen;
				}
				else {
					stop2 = start2 + slen;
				}
			}
			size_t get_len()
			{
				return stop-start+stop2-start2;
			}
			volatile T& operator [](size_t idx) volatile
			{
				std::make_signed<size_t>::type idx2 = idx-(stop-start);
				if(idx2<0)
					return start[idx];
				else
					return start2[idx2];
			}
			void put_bytes(const T* buff, size_t len,size_t at)
			{
				std::make_signed<size_t>::type at2 = at-(stop-start);
				if(at2 < 0)
				{
					std::make_signed<size_t>::type len1 = len-(stop-(start+at));
					if(len1<=0)
					{
						memcpy(const_cast<T*>(&start[at]),buff,len);
					}
					else
					{
						memcpy(const_cast<T*>(&start[at]),buff,len-len1);
						memcpy(const_cast<T*>(start2),&buff[len-len1],len1);
					}
				}
				else
				{
					memcpy(const_cast<T*>(&start2[at2]),buff,len);
				}
			}
		};
		volatile Tp ptr;
		const MemoryRegion & mr;

		Iterator(const MemoryRegion & _mr) : ptr(_mr.data_start), mr(_mr)
		{
		}
		Iterator(const Iterator & _mr) : ptr(_mr.ptr), mr(_mr.mr)
		{
		}
		inline void advance(size_t len) volatile
		{
			Tp tm = ptr+len;
			if(tm >= mr.data_end)
				tm -= mr.data_end-mr.data_start;
			ptr = tm;
		}
		inline void retreat(size_t len) volatile
		{
			Tp tm = ptr-len;
			if(tm < mr.data_start)
				tm =+ mr.data_end-mr.data_start;
			ptr = tm;
		}
	};

	Iterator wrptr;
	Iterator rdptr;

public:
	Fifo()
	: wrptr(mr)
	, rdptr(mr)
	{};

	inline void clear() volatile
	{
		wrptr.ptr = data;
		rdptr.ptr = data;
	}

	inline void clear_from_rd() volatile
	{
		rdptr.ptr = wrptr.ptr;
	}

	inline typename Iterator::Dereference get_wrbuff()
	{
		typename Iterator::Dereference buff = {wrptr};
		buff.set_len(fifo_free());
		return buff;
	}

	inline void commit_wrbuff(size_t len) volatile
	{
		wrptr.advance(len);
	}

	inline typename Iterator::Dereference get_rdbuff()
	{
		typename Iterator::Dereference buff = {rdptr};
		buff.set_len(fifo_check());
		return buff;
	}

	inline void commit_rdbuff(size_t len) volatile
	{
		rdptr.advance(len);
	}

	inline bool is_fifo_full() volatile
	{
		Iterator tmrd = wrptr;
		tmrd.advance();
		return tmrd.ptr == rdptr.ptr;
	}

	inline bool is_fifo_empty() volatile
	{
		return rdptr.ptr == wrptr.ptr;
	}

	inline size_t fifo_check() volatile
	{
		std::make_signed<size_t>::type ret = (wrptr.ptr - rdptr.ptr);
		if (ret < 0) ret +=SIZE;
		return ret;
	}

	inline size_t fifo_free() volatile
	{
		return SIZE-1-fifo_check();
	}
};
