

.PHONY: clean all default

THIS_DIR := $(dir $(abspath $(firstword $(MAKEFILE_LIST))))

default: optionrom.bin

all: default

HEADERS = inlines.h biosint.h
HEADERS_TARGETS = $(addprefix $(THIS_DIR),$(HEADERS))

%.obj : ${THIS_DIR}%.c $(HEADERS_TARGETS)
	cd ${THIS_DIR}; \
	wcc -0 -wx -s -oatx -hd -d0 -ms -zu -za99 -fo$(CURDIR)/$@ $<

%.obj : ${THIS_DIR}%.S 
	cd ${THIS_DIR}; \
	wasm -0 -wx -ms -fo$(CURDIR)/$@ $<


optionrom.bin: main.obj init.obj print.obj ${THIS_DIR}link.lnk
	wlink name $@ @${THIS_DIR}link.lnk
	${THIS_DIR}checksum.py $@
