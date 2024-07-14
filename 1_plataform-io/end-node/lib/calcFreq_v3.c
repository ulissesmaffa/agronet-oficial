#include <stdio.h>
#include <stdbool.h>

#define ARRAY_SIZE_1 560
#define ARRAY_SIZE_2 649
#define ARRAY_SIZE_3 502

#define AMOSTRAGEM_US 10

/*
Sinal imperfeito que começa com zero
Tem que ser considerado apenas os 500 valores que serão contabilizados após os 60 zeros
 60 | 391 | 109 
 0  |  1  |  0
*/
void create_signal_1(int signal[]) {
    int i;
    for(i=0;i<60;i++){//coluna 1
        signal[i]=0;
    }
    for(i=60;i<60+391;i++){//coluna 2
        signal[i]=1;
    }
    for(i=60+391;i<ARRAY_SIZE_1-1;i++) {//coluna 3
        signal[i]=0;
    }
    signal[ARRAY_SIZE_1-1]=1;
}
/*
Sinal imperfeito que começa com 1
Tem que ser considerado apenas os 500 valores que serão contabilizados após 40 registros de 1 e 109 registros de zero
 40 | 109 | 391 | 109 
 1  |  0  |  1  |  0
*/
void create_signal_2(int signal[]) {
    int i;
    for(i=0;i<40;i++){//coluna 1
        signal[i]=1;
    }
    for(i=40;i<40+109;i++){//coluna 2
        signal[i] = 0;
    }
    for(i=40+109;i<40+109+391;i++){//coluna 3
        signal[i]=1;
    }
    for(i=40+109+391;i<ARRAY_SIZE_2;i++){//coluna 4
        signal[i] = 0;
    }
}
/*
Sinal perfeito começando com 1
500 amostras X 10 us = 5.000 us = 200Hz
 391 | 109 
  1  |  0
*/
void create_signal_3(int signal[]) {
    int i;
    signal[0]=0;
    for(i=1;i<391;i++) {
        signal[i]=1;
    }
    for(i=391;i<ARRAY_SIZE_3-1;i++) {
        signal[i]=0;
    }
    signal[ARRAY_SIZE_3-1]=1;
}

void print_signal(int *signal, int size) {
    for(int i = 0; i < size; i++) {
        printf("%d ", signal[i]);
    }
    printf("\n");
}

void calc_frequency(int *signal, int size){
    int i=0,aux,count_1=0,count_0=0,seq=0,freq_count=0;
    float duty,freq,freq_before=-1.0,freq_avg,T,total_freq=0.0;;
    bool cut=true;

    aux=signal[i];
    for(i=0;i<size;i++){
        if(aux!=signal[i]){
            if(cut){
                printf("Corte:%i\n",i);
                cut=false;
                count_0=0;
                count_1=0;
            }else{
                seq++;
                if(seq==2){
                    printf("count_0: %i - count_1: %i\n",count_0,count_1);
                    T=(count_0+count_1);
                    duty=((float)count_1/T)*100;
                    freq=1/((T/1000000.0)*AMOSTRAGEM_US);
                    total_freq += freq; // Acumular todas as frequências
                    freq_count++;
                    if(freq_before != -1.0) {
                        freq_avg = (freq + freq_before) / 2;
                    } else {
                        freq_avg = freq;
                    }
                    printf("freq:%.3fHz - freq_avg:%.3fHz -  período: %.2f - duty: %.2f\n\n", freq, freq_avg, T, duty);
                    freq_before = freq;
                    count_0 = 0;
                    count_1 = 0;
                    seq = 0;
                }
            }
        }
        aux=signal[i];
        if(aux==0){
            count_0++;
        }else{
            count_1++;
        }
    }
    // Calcular a média geral das frequências
    if(freq_count > 0) {
        freq_avg = total_freq / freq_count;
    } else {
        freq_avg = 0;
    }
    printf("freq_avg:%.3fHz\n", freq_avg);
}

void calc_frequency2(int *i, int *aux, int *count_1, int *count_0, int *seq, int *freq_count, 
                     float *duty, float *freq, float *freq_before, float *freq_avg, float *T, float *total_freq, 
                     bool *cut, int signal_val){

    if(*aux!=signal_val){
        if(*cut){
            // printf("Corte:%i\n",*i);
            *cut=false;
            *count_0=0;
            *count_1=0;
        }else{
            (*seq)++;
            if(*seq==2){
                // printf("count_0: %i - count_1: %i\n",*count_0,*count_1);
                *T=(*count_0+*count_1);
                *duty=((float)*count_1 / *T)*100;
                *freq=1/((*T/1000000.0)*AMOSTRAGEM_US);
                *total_freq += *freq; // Acumular todas as frequências
                (*freq_count)++;
                if(*freq_before != -1.0) {
                    *freq_avg = (*freq + *freq_before) / 2;
                } else {
                    *freq_avg = *freq;
                }
                // printf("freq:%.3fHz - freq_avg:%.3fHz -  período: %.2f - duty: %.2f\n\n", *freq, *freq_avg, *T, *duty);
                *freq_before = *freq;
                *count_0 = 0;
                *count_1 = 0;
                *seq = 0;
            }
        }
    }

    *aux=signal_val;
    if(*aux==0){
        (*count_0)++;
    }else{
        (*count_1)++;
    }

    // Calcular a média geral das frequências
    if(*freq_count > 0) {
        *freq_avg = *total_freq / *freq_count;
    } else {
        *freq_avg = 0;
    }
    // printf("freq_avg:%.3fHz\n", *freq_avg);
}

int main() {
    int signal_1[ARRAY_SIZE_1];
    int signal_2[ARRAY_SIZE_2];
    int signal_3[ARRAY_SIZE_3];
    int signal_4[27]={0,0,0,1,1,1,1,0,0,1,1,1,0,0,1,1,1,1,1,0,1,1,1,1,0,0,1};
    int j;

    create_signal_1(signal_1);
    create_signal_2(signal_2);
    create_signal_3(signal_3);

    printf("Sinal 1:\n");
    print_signal(signal_1, ARRAY_SIZE_1);
    calc_frequency(signal_1,ARRAY_SIZE_1);

    printf("Sinal 2:\n");
    print_signal(signal_2, ARRAY_SIZE_2);
    calc_frequency(signal_2,ARRAY_SIZE_2);

    printf("Sinal 3:\n");
    print_signal(signal_3, ARRAY_SIZE_3);
    calc_frequency(signal_3,ARRAY_SIZE_3);

    printf("Sinal 4:\n");
    print_signal(signal_4,27);
    calc_frequency(signal_4,27);

    printf("\nALGORITMO NOVO:\n");
    /*Variáveis Globais*/
    int i=0,aux,count_1=0,count_0=0,seq=0,freq_count=0;
    float duty,freq,freq_before=-1.0,freq_avg,T,total_freq=0.0;
    bool cut=true;

    aux=signal_4[0];
    //Rodar esse for 15.000 vezes
    for(j=0;j<27;j++){
        calc_frequency2(&i, &aux, &count_1, &count_0, &seq, &freq_count, 
                    &duty, &freq, &freq_before, &freq_avg, &T, &total_freq, 
                    &cut, signal_4[j]);
    }
    printf("freq_avg:%.3fHz\n", freq_avg);

    return 0;
}
