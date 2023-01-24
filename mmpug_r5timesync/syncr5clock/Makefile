all:
	g++ -std=c++11 -fPIC -O2 -g -c -I include src/timesyncdata.cpp -o timesyncdata.o
	g++ -lrt -shared timesyncdata.o -o libtimesyncdata.so
	g++ -std=c++11 -L. -I include src/readclocks.cpp -o syncr5clock -ltimesyncdata -lrt 
	g++ -std=c++11 -L. -I include src/sampleshmreader.cpp -o sampleshmreader -ltimesyncdata -lrt 
	gcc src/comparepps.c -o gpio-int
	
install:
	cp libtimesyncdata.so /usr/lib/
	cp include/timesyncdata.h /usr/include/
	cp syncr5clock /usr/bin/syncr5clock
	cp service/syncr5clock.service /etc/systemd/system/
	systemctl enable syncr5clock.service
