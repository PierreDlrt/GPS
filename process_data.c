#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>


struct coordinate {
	uint8_t deg;
	uint8_t min;
	uint16_t sec;
	bool sig;
} lon, lat;

int main(int argc, char *argv[])
{
	
	printf("0\n");
    uint8_t strData[150] = {'\0'};
	uint8_t temp[150] = {'\0'};

	uint8_t data0 = '0';
	uint8_t data1 = '1';
	uint8_t data2 = '2';
	uint8_t data4 = '4';
	uint8_t data6 = '6';
	uint8_t dataA = 'A';
	uint8_t dataW = 'W';
	uint8_t comma = ',';
	uint8_t dot = '.';

	uint8_t tmp[10], tok[10];

	strncat(strData, &dataA, 1);
	strncat(strData, &comma, 1);
	strncat(strData, &data4, 1);
	strncat(strData, &data0, 1);
	strncat(strData, &data4, 1);
	strncat(strData, &data2, 1);
	strncat(strData, &dot, 1);
	strncat(strData, &data6, 1);
	strncat(strData, &data1, 1);
	strncat(strData, &data4, 1);
	strncat(strData, &data6, 1);
	strncat(strData, &comma, 1);
	strncat(strData, &dataW, 1);
	
    memcpy(temp, strData, strlen(strData)+1);

	memset(strData, '\0', 150*sizeof(uint8_t));
	printf("%s\n",temp);

 	if(*strtok(temp, ".,")=='A'){ 
		printf("if\n");
		strcpy(tok, strtok(NULL, ",.")); 
		printf("len=%d\n",strlen(tok));
		strncpy(tmp,tok,2);
		lat.deg = (uint8_t) atoi(tmp);
		//strcpy(tmp,tok+2,2)
		lat.min = (uint8_t) atoi(tok+2);

		printf("tmp=%s\n", tmp);
		printf("deg=%d\n", lat.deg);
		printf("min=%d\n", lat.min);

		strncpy(tok, strtok(NULL, ",."),3);
		tok[3] = '\0';
		lat.sec = (uint16_t) atoi(tok);
		printf("sec=%d\n", lat.sec);
		
		lat.sig = (*strtok(NULL, ",.")=='W'); // 0 = +, 1 = -
		printf("sig=%d\n", lat.sig);
	}


    return 0;
}
