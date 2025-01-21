CFLAGS	= 	-g -Wall -std=c99

MMGR_TAR	=	CIS3110-A1.tbz
MMGR		=	allocator
MMGR_OBJS	=	\
				mmanager_main.o \
				mmanager_model.o \
				mmanager_parse.o

$(MMGR) : $(MMGR_OBJS)
	$(CC) $(CFLAGS) -o $(MMGR) $(MMGR_OBJS)

tags ctags : dummy
	- ctags *.c

tar : dummy
	tar cvjf $(MMGR_TAR) *.c *.h makefile *.md *.txt
	if [ -d unpack ] ; then rm -rf unpack ; fi
	mkdir unpack
	( cd unpack ; tar xvf ../$(MMGR_TAR) )
	@ echo ""
	@ echo "Check the 'unpack' directory to see that it works."
	@ echo ""


clean :
	- rm -f $(MMGR)
	- rm -f $(MMGR_OBJS)

dummy :
