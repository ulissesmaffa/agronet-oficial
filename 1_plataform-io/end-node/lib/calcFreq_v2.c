#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define BUFFER_SIZE 20
#define AMOSTRAGEM_US 4000

void printVector(int *v, int size){
    int i;

    for(i=0;i<size;i++){
        printf("%i ",v[i]);
    }
    printf("\n");
}

void newBufferSize(int *buffer, int size, int *valueNewBufferSize, int *valueStartCutBuffer){
    bool ctr,changed;
    int qtd,i,startCut;

    //discartando inicio do sinal
    ctr=true;
    changed=false;
    i=0;

    while(ctr){
        if(buffer[i]!=buffer[i+1]){
            if(changed){
                ctr=false;
            }else{
                changed=true;
            }
        }
        i++;
    }
    startCut=i;
    qtd=size-i;

    //discartando final do sinal
    ctr=true;
    changed=false;
    i=size-1;
    
    //se sinal começa com o mesmo valor que termina
    if(buffer[0]==buffer[size-1]){
        printf("se sinal começa com o mesmo valor que termina\n");
        while(ctr){
            if(buffer[i]==buffer[i-1]){
                i--;
            }else{
                ctr=false;
            }
        }
    }else{//se o sinal começa com um valor diferente do que termina
        printf("se o sinal começa com um valor diferente do que termina\n");
        while(ctr){
            if(buffer[i]!=buffer[i-1]){
                if(changed){
                    ctr=false;
                }else{
                    changed=true;
                }
            }
            i--;
        }
    }
    qtd=qtd-(size-i);
    
    *valueNewBufferSize=qtd;
    *valueStartCutBuffer=startCut;
}

void insertNewBuffer(int *buffer, int *newBuffer, int size_b2, int startCut){
    int i;
    for(i=0;i<size_b2;i++){
        newBuffer[i]=buffer[startCut+i];
    }
}

float calculateFrequency(int *v, int size){
    bool ctr,changed;
    int i,group;
    float freq=0.0;

    ctr=true;
    changed=false;
    group=0;

    for(i=0;i<size;i++){
        if(v[i]!=v[i+1]){
            if(changed){
                changed=false;
                group++;
            }else{
                changed=true;
            }   
        }
        freq=freq+(AMOSTRAGEM_US/1000);
    }
    freq=freq/group;

    return freq;
}

int main(){
    int buffer[BUFFER_SIZE]={
        0,1,1,1,1,1,0,0,1,1,1,1,1,0,1,1,1,1,1,0
    };
    int *newBuffer,valueNewBufferSize,valueStartCutBuffer;
    float frequency;

    printVector(buffer,BUFFER_SIZE);
    newBufferSize(buffer,BUFFER_SIZE,&valueNewBufferSize,&valueStartCutBuffer);
    newBuffer=(int *) malloc(valueNewBufferSize * sizeof(int));
    insertNewBuffer(buffer,newBuffer,valueNewBufferSize,valueStartCutBuffer);
    printVector(newBuffer,valueNewBufferSize);
    frequency=calculateFrequency(newBuffer,valueNewBufferSize);
    printf("Frequencia é: %.2fHz\n",frequency);

    return 0;
}