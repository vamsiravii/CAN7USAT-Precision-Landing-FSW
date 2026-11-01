FSW := ../CAN7USAT_UNO
SRCS := sil_main.cpp $(FSW)/navigation.cpp $(FSW)/guidance.cpp $(FSW)/flight_sm.cpp $(FSW)/gnc.cpp
HDRS := sim_world.h $(wildcard $(FSW)/*.h)

sil: $(SRCS) $(HDRS)
	g++ -O2 -std=c++17 -Wall -Wextra -I$(FSW) $(SRCS) -o $@

clean:
	rm -f sil *.csv

.PHONY: clean
