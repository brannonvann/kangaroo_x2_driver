// Speed Control Sample for Kangaroo
// Modified from Kangaroo Arduino library example SeedControl.ino

#include <Kangaroo.h>
#include <unistd.h>

using namespace std;

string port = "/dev/ttyUSB0";
unsigned long baud = 9600;
long loopMax = 2;
long loopCounter = 0; 

serial::Serial serial_port(port, baud, serial::Timeout::simpleTimeout(1000));
SerialStream stream(serial_port);
KangarooSerial  K(stream);
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');

void delay(int milliseconds)
{
    useconds_t microseconds = milliseconds * 1000;
    usleep(microseconds);
}

void setup()
{
  K1.start();
  K2.start();
}

void loop()
{

  long minimum = K1.getMin().value();
  long maximum = K1.getMax().value();
  long speed   = (maximum - minimum) / 10; // 1/10th of the range per second

  K1.s(speed);
  delay(1000);
  K1.s(0);
  delay(1000);
  
  K1.s(-speed);
  delay(1000);
  K1.s(0);
  delay(1000);

  minimum = K2.getMin().value();
  maximum = K2.getMax().value();
  speed   = (maximum - minimum) / 10; // 1/10th of the range per second

  K2.s(speed);
  delay(1000);
  K2.s(0);
  delay(1000);
  
  K2.s(-speed);
  delay(1000);
  K2.s(0);
  delay(1000);

  loopCounter += 1;
}

int main() 
{
    setup();    
    while (loopCounter < loopMax) {
        loop();
    }    
    return 0;
}
