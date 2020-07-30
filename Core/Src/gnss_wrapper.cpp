#include "zedf9p/gnss_wrapper.h"
#include "zedf9p/gnss.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// Inside this "extern C" block, I can implement functions in C++, which will externally
//   appear as C functions (which means that the function IDs will be their names, unlike
//   the regular C++ behavior, which allows defining multiple functions with the same name
//   (overloading) and hence uses function signature hashing to enforce unique IDs),

CSensors* copy_struct(){
	//float *aaa = &gnss_sensor.gnss.pos.alt;
	//float *bbb = &str_cop->pos.alt;
	//str_cop->pos->alt = &gnss_sensor.gnss.pos.alt;
	//&str_cop->pos.alt = &gnss_sensor.gnss.pos.alt;
	//&str_cop->pos.lat = &gnss_sensor.gnss.pos.lat;
	//&str_cop->pos.lon = &gnss_sensor.gnss.pos.lon;
	return &gnss_sensor;
}

CGNSS* newCGNSS(){

	return new CGNSS();
}

CSensors* newCSensors(){
	return new CSensors();
}

void handleGNSS_c(void){
	handleGNSS();
}


#ifdef __cplusplus
}
#endif


