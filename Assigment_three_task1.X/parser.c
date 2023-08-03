#include "parser.h"

int parse_byte(parser_state* ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$') {
                ps->state = STATE_TYPE;
                ps->index_type = 0;
            }
            break;
        case STATE_TYPE:
            if (byte == ',') {
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0; // initialize properly the index
            } else if (ps->index_type == 6) { // error! 
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;
			} else if (byte == '*') {
				ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_type[ps->index_type] = '\0';
				ps->msg_payload[0] = '\0'; // no payload
                return NEW_MESSAGE;
            } else {
                ps->msg_type[ps->index_type] = byte; // ok!
                ps->index_type++; // increment for the next time;
            }
            break;
        case STATE_PAYLOAD:
            if (byte == '*') {
                ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;
            } else if (ps->index_payload == 100) { // error
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;
            } else {
                ps->msg_payload[ps->index_payload] = byte; // ok!
                ps->index_payload++; // increment for the next time;
            }
            break;
    }
    return NO_MESSAGE;
}


int extract_numbers(char *string, double *num1, double *num2) {
    char str1[10], str2[10];
    int i, j, k, dot_count1, dot_count2;
    i = j = k = dot_count1 = dot_count2 = 0;

    // Extract the two numbers from the input string
    while(string[i] != ',' && string[i] != '\0') {
        str1[j++] = string[i++];
    }
    str1[j] = '\0';
    i++;
    while(string[i] != '\0') {
        str2[k++] = string[i++];
    }
    str2[k] = '\0';

    // Check if the numbers are integers or floating-point numbers
    for(i = 0; i < j; i++) {
        if(str1[i] == '.') {
            dot_count1++;
        }
    }
    for(i = 0; i < k; i++) {
        if(str2[i] == '.') {
            dot_count2++;
        }
    }

    if(dot_count1 == 0) {
        // Number is an integer
        *num1 = (float) atoi(str1);
    } else {
        // Number is a floating-point number
        *num1 = atof(str1);
    }
    if(dot_count2 == 0) {
        // Number is an integer
        *num2 = (float) atoi(str2);
    } else {
        // Number is a floating-point number
        *num2 = atof(str2);
    }
    return 1;
}


