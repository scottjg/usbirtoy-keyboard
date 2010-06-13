/*
*
*	USB infrared remote control receiver transmitter firmware v1.0
*	License: creative commons - attribution, share-alike 
*	Copyright Ian Lesnet 2010
*	http://dangerousprototypes.com
*
*/
int decodeNec(unsigned char *addr, unsigned char *data);
int decodeRC5(unsigned char *addr, unsigned char *data);
