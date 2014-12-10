void printScreen(String alt, String temp, String gyroX, String gyroY, String heading, String acceleration, String outsideTemp)
{
  lcd.home();
  String line1 = "Alti:";
  line1.concat(alt);
  line1.concat(" Head: ");
  line1.concat(heading);
  
  String line2 = "Temp:";
  line2.concat(outsideTemp);
  line2.concat(" Bump:");
  line2.concat(acceleration);
  
  String line3 = "Pitch:";
  line3.concat(gyroX);
  line3.concat("  "); 
  
  String line4 = "Roll: ";
  line4.concat(gyroY); 
  line4.concat("  "); 

  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
  lcd.setCursor(0,2);
  lcd.print(line3);
  lcd.setCursor(0,3);
  lcd.print(line4);
}
