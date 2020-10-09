#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


struct coordinate {
	uint8_t* deg;
	uint8_t* min;
} lon, lat;

int main(int argc, char *argv[])
{
	
	printf("0\n");
    uint8_t strData[150] = {'\0'};
	uint8_t temp[150] = {'\0'};

	uint8_t data1 = '1';
	uint8_t data1b = '2';
	uint8_t data2 = 'a';
	uint8_t data3 = '*';
	uint8_t data4 = 'G';
	uint8_t comma = ',';
	uint8_t dot = '.';

	strncat(strData, &data1, 1);
	strncat(strData, &data1b, 1);
	strncat(strData, &comma, 1);
	strncat(strData, &comma, 1);
	strncat(strData, &data2, 1);
	strncat(strData, &dot, 1);
	strncat(strData, &data3, 1);
	strncat(strData, &comma, 1);
	strncat(strData, &data4, 1);


    memcpy(temp, strData, strlen(strData)+1);

	memset(strData, '\0', 150*sizeof(uint8_t));
	printf("%s\n",temp);

    uint8_t *tok = strtok(temp, ","); 
	printf("len=%d\n",strlen(tok));	
	printf("sizeof=%d\n",sizeof(uint8_t));
	printf("size=%d\n",sizeof(*tok));
	//uint8_t *toke;


	/*if (!memcmp(token,"12",2)){
		printf("if\n");
		token = strtok(NULL, ",");
		printf("tok=%c\n",*token);
		toke = strtok(NULL, ","); 	
		printf("%s\n", toke); 
		/*while (token != NULL) { 
			printf("%s\n", token); 
			token = strtok(NULL, ","); 
		}
	}*/

	//struct coordinate lon = { strtok(NULL, ","), strtok(NULL, ",") };

	lon.deg = strtok(NULL, ",");
	lon.min = strtok(NULL, ",.");

	printf("float=%d\n", sizeof(float));

	printf("deg=%c\n", *lon.deg);
	printf("min=%s\n", lon.min);
	

    return 0;
}
