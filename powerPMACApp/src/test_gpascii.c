//#include <conio.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
  int JogVel;
  int Pos;
} Foo;

void reply(const char *str)
{
  char buffer[1024];
  sprintf(buffer, "%s\n%c", str, 0x07);
  printf(buffer);  
}

int main(int argc, char *argv[])
{
  int c;
  int counter = 0;
  char cmd[1024];
  printf("STDIN Open for ASCII Input\n");
  double pvar[12000];
  int index;
  for (index = 0; index < 12000; index++){
    pvar[index] = 0.0;
  }

  counter = 0;
  do {
    counter = 0;
    do {
      c = getchar();
      cmd[counter] = c;
      counter++;
    } while(c != '\n');
    // Append the terminator
    cmd[counter-1] = '\0';
    // Check for commands here
    if (!strcmp(cmd, "vers")){
      reply("1.467");
    } else if (!strcmp(cmd, "cpu")){
      reply("PowerPC,460EX");
    } else if (!strcmp(cmd, "size")){
      reply("Program Buffer = 16777216\nUser Buffer = 1048576\nTable Buffer = 1048576\nLookahead Buffer = 16777216\nSynOps Buffer = 1048576\nSymbols Buffer = 1048576");
    } else if (cmd[0] == 'P'){
      int p = 0;
      int cnt = 0;
      int match = 0;
      double val = 0.0;
      char buf[512];
      sscanf(cmd,"P%d",&p);
      for (cnt = 0; cnt < strlen(cmd); cnt++){
        if (cmd[cnt] == '='){
          match = cnt;
        }
      }
      if (match != 0){
        sscanf(&cmd[match+1], "%lf", &val);
        pvar[p] = val;
        reply("");
      } else {
        sprintf(buf, "%lf", pvar[p]);
        reply(buf);
      }
    }
    // Quit if we get a quit
  } while(strcmp(cmd, "quit"));
  return 0;
}

