build:
	rm -f application.exe
	g++ -std=c++2a -Wall application.cpp dist.cpp osm.cpp tinyxml2.cpp -o application.exe

run:
	./application.exe

buildtest:
	rm -f testing.exe
	g++ -std=c++2a -Wall testing.cpp -o testing.exe

runtest:
	./testing.exe

clean:
	rm -f application.exe	

valgrind:
	valgrind --tool=memcheck --leak-check=full --track-origins=yes ./application.exe

runsolution:
	./maps_solution.out