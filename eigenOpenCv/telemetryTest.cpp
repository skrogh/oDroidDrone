#include "telemetry.hpp"

int main( int argc, char** argv )
{
	Telemetry telemetry( 30000 );
	while(1){
		char buf[256];
		telemetry.send( fgets( buf, 256, stdin ), 256 );

	}
}