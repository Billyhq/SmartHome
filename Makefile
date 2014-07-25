all:
	gcc -o obj/server server.c
	gcc -o obj/client client.c
clean:
	rm -f obj/server obj/client
