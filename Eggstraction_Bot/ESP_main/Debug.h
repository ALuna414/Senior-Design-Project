#ifdef WEB_SERIAL_ENABLED

String debug_string;

void debugPrint(String& s){ debug_string += s;}
void debugPrint(int i){ debug_string += String(i);}
void debugPrint(float f){ debug_string += String(f);}

void debugPrintln(String& s){ debug_string += s + '\n';}
void debugPrintln(int i){ debug_string += String(i) + '\n';}
void debugPrintln(float f){ debug_string += String(f) + '\n';}

void debugTransmit()
{
  WebSerial.println(debug_string);
  debug_string = "";
}

#else

void debugPrint(String& s){ Serial.print(s);}
void debugPrint(int i){ Serial.print(i);}
void debugPrint(float f){ Serial.print(f);}

void debugPrintln(String& s){ Serial.println(s);}
void debugPrintln(int i){ Serial.println(i);}
void debugPrintln(float f){ Serial.println(f);}

void debugTransmit(){}

#endif
