# Host test for the hub's link key handshake: make && ./test_linkkey
test_linkkey: test_linkkey.c ../main/linkkey.c ../main/linkkey.h include/mbedtls/md.h
	$(CC) -O1 -g -Wall -Wextra -Iinclude -I../main -o $@ test_linkkey.c ../main/linkkey.c -lcrypto
clean:
	rm -f test_linkkey
