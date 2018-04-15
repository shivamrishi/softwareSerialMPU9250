#include<stdio.h>

int main(){
int i = 0;
FILE* fp, *hxl[32],*hxh[32],*hyl[32],*hyh[32];
char buffer[80];
for(i = 0;i<32;i++){
sprintf(buffer, "hxl%d.txt", i);
hxl[i] = fopen(buffer,"w");
}
for( i = 0;i<32;i++){
sprintf(buffer, "hxh%d.txt", i);
hxh[i] = fopen(buffer,"w");
}
for( i = 0;i<32;i++){
sprintf(buffer, "hyl%d.txt", i);
hyl[i] = fopen(buffer,"w");
}
for( i = 0;i<32;i++){
sprintf(buffer, "hyh%d.txt", i);
hyh[i] = fopen(buffer,"w");
}
char _buf[80];
i = 0;
int j;
int count = 400;// number of bytes
fp = fopen("sensorLogs.txt", "r");
    i =0;
    while(i <32){
    for( j = 0;j< count; j++){
    fscanf(fp,"%s",_buf);
        fprintf(hxl[i],"%s\t", _buf);
    fscanf(fp,"%s",_buf);
        fprintf(hxh[i],"%s\t", _buf);
    fscanf(fp,"%s",_buf);
        fprintf(hyl[i],"%s\t",_buf);
    fscanf(fp,"%s",_buf);
        fprintf(hyh[i],"%s\t", _buf);
    }  i++;
    }
return 0;
}
