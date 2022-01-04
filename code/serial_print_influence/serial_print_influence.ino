
int counter_print, counter_noprint;
unsigned long start_time, now;
#define RUNTIME 5000 //ms

void setup() {
  Serial.begin(9600);

  
  // put your setup code here, to run once:
  
  start_time = millis();
  while((millis() - start_time) < RUNTIME){
    counter_print++;
    Serial.println("Some characters here and there");
  }

  start_time = millis();
  while((millis() - start_time) < RUNTIME){
    counter_noprint++;
  }  

  Serial.print("Iterations with print: ");Serial.println(counter_print);
  Serial.print("Iterations without print: ");Serial.println(counter_noprint);
}

void loop() {
  // put your main code here, to run repeatedly:
  

}
