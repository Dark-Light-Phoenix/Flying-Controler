#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>

const char *ssid = "STM";
const char *password = "00000000";

ESP8266WebServer server(80);

String xDataAcc, yDataAcc, zDataAcc;
String xDataGyro, yDataGyro, zDataGyro;
String kalmanXDataAcc, kalmanYDataAcc, kalmanZDataAcc;
String kalmanXDataGyro, kalmanYDataGyro, kalmanZDataGyro;
String StartPressure, StartTemperature, StartHeight;
String Pressure, Temperature, Height, DeltaHeight;

bool dataReceived = false;

volatile int reload = 0;

void recieveData_(void);

void handleRoot() 
{
  receiveData_();
  
  String page = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='5'><title>Flyight Controller</title></head><body>";
  page += "<h1>You are connected</h1>";
  page += "<p>Data received: " + String(dataReceived ? "Yes" : "No") + "</p>";
  page += "<p>Serial.available(): " + String(Serial.available()) + "</p>";
  
  page += "<h2><b>Table 1: Data of Accelerometer and Gyroscope without Kalman and with</b></h2>";
  page += "<table border='1'><colgroup><col style='width: 80px'><col style='width: 65px'><col style='width: 65px'><col style='width: 65px'><col style='width: 65px'><col style='width: 65px'><col style='width: 65px'></colgroup>";
  page += "<tr><td></td><td style='text-align: center;'>X</td><td style='text-align: center;'>Y</td><td style='text-align: center;'>Z</td><td>KalmanX</td><td>KalmanY</td><td>KalmanZ</td></tr>";
  page += "<tr><td>Accelerometer</td><td id='xAcc'>" + xDataAcc + "</td><td id='yAcc'>" + yDataAcc + "</td><td id='zAcc'>" + zDataAcc + "</td><td id='kalmanXAcc'>" + kalmanXDataAcc + "</td><td id='kalmanYAcc'>" + kalmanYDataAcc + "</td><td id='kalmanZAcc'>" + kalmanZDataAcc + "</td></tr>";
  page += "<tr><td>Gyroscope</td><td id='xGyro'>" + xDataGyro + "</td><td id='yGyro'>" + yDataGyro + "</td><td id='zGyro'>" + zDataGyro + "</td><td id='kalmanXGyro'>" + kalmanXDataGyro + "</td><td id='kalmanYGyro'>" + kalmanYDataGyro + "</td><td id='kalmanZGyro'>" + kalmanZDataGyro + "</td></tr></table>";
  
  page += "<h2><b>Table 2: Data of start pressure, temperature and height which is chosen as the notation of the 0th height</b></h2>";
  page += "<table border='1'><colgroup><col style='width: 80px;'><col style='width: 80px;'><col style='width: 80px;'></colgroup>";
  page += "<tr><td style='text-align: center;'>StartPressure</td><td style='text-align: center;'>StartTemperature</td><td style='text-align: center;'>StartHeight</td></tr>";
  page += "<tr><td id='StartPressure'>" + StartPressure + "</td><td id='StartTemperature'>" + StartTemperature + "</td><td id='StartHeight'>" + StartHeight + "</td></tr></table>";
  
  page += "<h2><b>Table 3: Data of next measurement of pressure, temperature and height, which are compared to start variables and show difference between start height and present height as DeltaHeight</b></h2>";
  page += "<table border='1'><colgroup><col style='width: 65px;'><col style='width: 65px;'><col style='width: 65px;'><col style='width: 65px;'></colgroup>";
  page += "<tr><td style='text-align: center;'>Pressure</td><td style='text-align: center;'>Temperature</td><td style='text-align: center;'>Height</td><td style='text-align: center;'>DeltaHeight</td></tr>";
  page += "<tr><td id='Pressure'>" + Pressure + "</td><td id='Temperature'>" + Temperature + "</td><td id='Height'>" + Height + "</td><td id='DeltaHeight'>" + DeltaHeight + "</td></tr></table>";
  page += "<p>reload = " + String (reload) + "</p>";
  
  page += "<script>";
  page += "function fetchData() {";
  page += "  var xhr = new XMLHttpRequest();";
  page += "  xhr.onreadystatechange = function() {";
  page += "    if (this.readyState == 4 && this.status == 200) {";
  page += "      var data = JSON.parse(this.responseText);";
  page += "      document.getElementById('xAcc').innerHTML = data.xAcc;";
  page += "      document.getElementById('yAcc').innerHTML = data.yAcc;";
  page += "      document.getElementById('zAcc').innerHTML = data.zAcc;";
  page += "      document.getElementById('kalmanXAcc').innerHTML = data.kalmanXAcc;";
  page += "      document.getElementById('kalmanYAcc').innerHTML = data.kalmanYAcc;";
  page += "      document.getElementById('kalmanZAcc').innerHTML = data.kalmanZAcc;";
  page += "      document.getElementById('xGyro').innerHTML = data.xGyro;";
  page += "      document.getElementById('yGyro').innerHTML = data.yGyro;";
  page += "      document.getElementById('zGyro').innerHTML = data.zGyro;";
  page += "      document.getElementById('kalmanXGyro').innerHTML = data.kalmanXGyro;";
  page += "      document.getElementById('kalmanYGyro').innerHTML = data.kalmanYGyro;";
  page += "      document.getElementById('kalmanZGyro').innerHTML = data.kalmanZGyro;";
  page += "    }";
  page += "  };";
  page += "  xhr.open('GET', '/data1', true);";
  page += "  xhr.send();";
  page += "}";
  
  page += "function fetchData2() {";
  page += "  var xhr2 = new XMLHttpRequest();";
  page += "  xhr2.onreadystatechange = function() {";
  page += "    if (this.readyState == 4 && this.status == 200) {";
  page += "      var data = JSON.parse(this.responseText);";
  page += "      document.getElementById('Pressure').innerHTML = data.Pressure;";
  page += "      document.getElementById('Temperature').innerHTML = data.Temperature;";
  page += "      document.getElementById('Height').innerHTML = data.Height;";
  page += "      document.getElementById('DeltaHeight').innerHTML = data.DeltaHeight;";
  page += "    }";
  page += "  };";
  page += "  xhr2.open('GET', '/data3', true);";
  page += "  xhr2.send();";
  page += "}";
  
  page += "setInterval(fetchData, 5000);";
  page += "setInterval(fetchData2, 5000);";
  page += "</script>";
  page += "</body></html>";
  server.send(200, "text/html", page);

  Serial.println("reload = " + String (reload));
}

void handleData1() {
  String jsonData = "{";
  jsonData += "\"xAcc\":\"" + xDataAcc + "\",";
  jsonData += "\"yAcc\":\"" + yDataAcc + "\",";
  jsonData += "\"zAcc\":\"" + zDataAcc + "\",";
  jsonData += "\"kalmanXAcc\":\"" + kalmanXDataAcc + "\",";
  jsonData += "\"kalmanYAcc\":\"" + kalmanYDataAcc + "\",";
  jsonData += "\"kalmanZAcc\":\"" + kalmanZDataAcc + "\",";
  jsonData += "\"xGyro\":\"" + xDataGyro + "\",";
  jsonData += "\"yGyro\":\"" + yDataGyro + "\",";
  jsonData += "\"zGyro\":\"" + zDataGyro + "\",";
  jsonData += "\"kalmanXGyro\":\"" + kalmanXDataGyro + "\",";
  jsonData += "\"kalmanYGyro\":\"" + kalmanYDataGyro + "\",";
  jsonData += "\"kalmanZGyro\":\"" + kalmanZDataGyro + "\"";
  jsonData += "}";
  server.send(200, "application/json", jsonData);
}

void handleData3() {
  String jsonData = "{";
  jsonData += "\"Pressure\":\"" + Pressure + "\",";
  jsonData += "\"Temperature\":\"" + Temperature + "\",";
  jsonData += "\"Height\":\"" + Height + "\",";
  jsonData += "\"DeltaHeight\":\"" + DeltaHeight + "\"";
  jsonData += "}";
  server.send(200, "application/json", jsonData);
}

void setup() 
{
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Configuring access point...");

  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  server.on("/", handleRoot);
  server.on("/data1", handleData1);
  server.on("/data3", handleData3);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() 
{
  server.handleClient();
  delay(100);
}

void receiveData_() 
{
  if (Serial.available() > 0) 
  {
    String data = Serial.readStringUntil('\n');
    Serial.println("Data received: " + data);

    int xIndex = data.indexOf('X');
    int yIndex = data.indexOf('Y');
    int zIndex = data.indexOf('Z');
    int gxIndex = data.indexOf("GX");
    int gyIndex = data.indexOf("GY");
    int gzIndex = data.indexOf("GZ");
    int kaxIndex = data.indexOf("KAX");
    int kayIndex = data.indexOf("KAY");
    int kazIndex = data.indexOf("KAZ");
    int kgxIndex = data.indexOf("KGX");
    int kgyIndex = data.indexOf("KGY");
    int kgzIndex = data.indexOf("KGZ");
    int spIndex = data.indexOf("SP");
    int stIndex = data.indexOf("ST");
    int shIndex = data.indexOf("SH");
    int pIndex = data.indexOf("P");
    int tIndex = data.indexOf("T");
    int hIndex = data.indexOf("H");
    int dhIndex = data.indexOf("DH");
    reload++;

    if (xIndex != -1 && yIndex != -1) 
    {
      xDataAcc = data.substring(xIndex + 1, yIndex);
    }
    
    if (yIndex != -1 && zIndex != -1) 
    {
      yDataAcc = data.substring(yIndex + 1, zIndex);
    }
    
    if (zIndex != -1 && gxIndex != -1) 
    {
      zDataAcc = data.substring(zIndex + 1, gxIndex);
    }
    
    if (gxIndex != -1 && gyIndex != -1) 
    {
      xDataGyro = data.substring(gxIndex + 2, gyIndex);
    }
    
    if (gyIndex != -1 && gzIndex != -1) 
    {
      yDataGyro = data.substring(gyIndex + 2, gzIndex);
    }
    
    if (gzIndex != -1 && kaxIndex != -1) 
    {
      zDataGyro = data.substring(gzIndex + 2, kaxIndex);
    }
    
    if (kaxIndex != -1 && kayIndex != -1) 
    {
      kalmanXDataAcc = data.substring(kaxIndex + 3, kayIndex);
    }
    
    if (kayIndex != -1 && kazIndex != -1) 
    {
      kalmanYDataAcc = data.substring(kayIndex + 3, kazIndex);
    }
    
    if (kazIndex != -1 && kgxIndex != -1) 
    {
      kalmanZDataAcc = data.substring(kazIndex + 3, kgxIndex);
    }
    
    if (kgxIndex != -1 && kgyIndex != -1) 
    {
      kalmanXDataGyro = data.substring(kgxIndex + 3, kgyIndex);
    }
    
    if (kgyIndex != -1 && kgzIndex != -1) 
    {
      kalmanYDataGyro = data.substring(kgyIndex + 3, kgzIndex);
    }
    
    if (kgzIndex != -1 && shIndex != -1) 
    {
      kalmanZDataGyro = data.substring(kgzIndex + 3, spIndex);
    }

    if (spIndex != -1 && stIndex != -1) 
    {
      StartPressure = data.substring(spIndex + 2, stIndex);
    }

    if (stIndex != -1 && shIndex != -1) 
    {
      StartTemperature = data.substring(stIndex + 2, shIndex);
    }
    
    if (shIndex != -1 && pIndex != -1) 
    {
      StartHeight = data.substring(shIndex + 2, pIndex);
    }

    if (pIndex != -1 && tIndex != -1) 
    {
      Pressure = data.substring(pIndex + 1, tIndex);
    }

    if (tIndex != -1 && hIndex != -1) 
    {
      Temperature = data.substring(tIndex + 1, hIndex);
    }

    if (hIndex != -1 && dhIndex != -1) 
    {
      Height = data.substring(hIndex + 1, dhIndex);
      DeltaHeight = data.substring(dhIndex + 2);
    }
    
    dataReceived = true;
  }
  else
  {
    dataReceived = false;
  }
}
