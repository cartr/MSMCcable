OBJS = serxfer.o main.o multi.o

CCOPTS = -g

gbl:	$(OBJS)
	gcc $(OBJS) -o gbl

main.o:	main.c 2ndloader/loader.h

clean:	
	rm *.o gbl
