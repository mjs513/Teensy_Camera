#if defined(USE_SDCARD)

void storage_configure()
{
  const char *pn;
  DateTimeFields date;
  breakTime(Teensy3Clock.get(), date);
  const char *monthname[12]={
    "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  Serial.printf("Date: %u %s %u %u:%u:%u\n",
    date.mday, monthname[date.mon], date.year+1900, date.hour, date.min, date.sec);


  // Lets add the Prorgram memory version:
  // checks that the LittFS program has started with the disk size specified
  // Lets add the Program memory version:
  // checks that the LittFS program has started with the disk size specified
  if (lfsProg.begin(file_system_size)) {
    MTP.addFilesystem(lfsProg, "PgmIndx");
  } else {
    Serial.println("Error starting Program Flash storage");
  }

  #if defined SD_SCK
    SPI.setMOSI(SD_MOSI);
    SPI.setMISO(SD_MISO);
    SPI.setSCK(SD_SCK);
  #endif

  for (uint8_t i = 0; i < nsd; i++) {
    if(cs[i] != BUILTIN_SDCARD) {
      pinMode(cs[i], OUTPUT);
      digitalWrite(cs[i], HIGH);
    }
/*
      sd_media_present_prev[i] = sdx[i].begin(cs[i]);
      if (cdPin[i] != 0xff) sdx[i].setMediaDetectPin(cdPin[i]);
        MTP.addFilesystem(sdx[i], sd_str[i]);
      if (sd_media_present_prev[i]) {
        uint64_t totalSize = sdx[i].totalSize();
        uint64_t usedSize  = sdx[i].usedSize();
        Serial.printf("SD Storage %d %d %s ",i,cs[i],sd_str[i]); 
        Serial.print(totalSize); Serial.print(" "); Serial.println(usedSize);
      }
    }
    elapsed_millis_since_last_sd_check = 0;
*/
  if (sdx[i].begin(cs[i])) {
  Serial.println("Added SD card using built in SDIO, or given SPI CS");
  } else {
  Serial.println("No SD Card SDIO or SPI CS");
  }
  MTP.addFilesystem(sdx[i], sd_str[i]);
  uint64_t totalSize = sdx[i].totalSize();
  uint64_t usedSize  = sdx[i].usedSize();
  Serial.printf("SD Storage %d %d %s ",i,cs[i],sd_str[i]); 
  Serial.print(totalSize); Serial.print(" "); Serial.println(usedSize);
}
  
Serial.println("\nSD Setup done");

}


void listFiles()
{
  Serial.print("\n Space Used = ");
  Serial.println(myfs->usedSize());
  Serial.print("Filesystem Size = ");
  Serial.println(myfs->totalSize());

  printDirectory(myfs);
}

void eraseFiles()
{
  //Serial.println("Formating not supported at this time");
  Serial.println("\n*** Erase/Format started ***");
  myfs->format(1, '.', Serial);
  Serial.println("Completed, sending device reset event");
  MTP.send_DeviceResetEvent();
}

void printDirectory(FS *pfs) {
  Serial.println("Directory\n---------");
  printDirectory(pfs->open("/"), 0);
  Serial.println();
}

void printDirectory(File dir, int numSpaces) {
  while (true) {
    File entry = dir.openNextFile();
    if (! entry) {
      //Serial.println("** no more files **");
      break;
    }
    printSpaces(numSpaces);
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numSpaces + 2);
    } else {
      // files have sizes, directories do not
      printSpaces(36 - numSpaces - strlen(entry.name()));
      Serial.print("  ");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void printSpaces(int num) {
  for (int i = 0; i < num; i++) {
    Serial.print(" ");
  }
}


uint32_t CommandLineReadNextNumber(int &ch, uint32_t default_num) {
  while (ch == ' ') ch = Serial.read();
  if ((ch < '0') || (ch > '9')) return default_num;

  uint32_t return_value = 0;
  while ((ch >= '0') && (ch <= '9')) {
    return_value = return_value * 10 + ch - '0';
    ch = Serial.read();
  }
  return return_value;
}

#endif