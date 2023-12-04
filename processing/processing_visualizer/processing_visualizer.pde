import processing.serial.*;


Serial myPort; // declare port object

int Setpoint = 15, Input = 0, Output = 0;

int [] Inputs = new int[20];
int [] Outputs = new int[20];

void add_sample(int sample, int [] arr, int sz){
  for(int i = 0; i < sz - 1; ++i){
    arr[i] = arr[i+1];
  }
  arr[sz-1] = sample;
}

void draw_h_line(int pos, int min, int max, int r, int g, int b, String label){
  double yScale = height/((double)(max-min))/2.0;
  pos = (int)((double)pos*yScale);
  strokeWeight(5);
  stroke(r,g,b);
  line(0.f,(float)(height/2.f-(float)pos),(float)width,(float)(height/2.f-(float)pos));
  textSize(20);
  fill(r,g,b);
  text(label,12,(float)(height/2.f-(float)pos)+30);
}

void setup(){
  size(800,600,P2D);
  frameRate(60);
  
  myPort = new Serial(this, Serial.list()[0], 9600); // initialize port at 9600 baud, will throw an error if no port detected
  printArray(Serial.list());
}

void draw(){
  background(0xFF);
  draw_h_line(Setpoint,0, 27,255,0,0, "setpoint: " + Setpoint + "cm");
}
