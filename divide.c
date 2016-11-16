#include <stdio.h>


// Works exactly like elementary school division, except instead of having to guess a number betwee 0 and 9, you see if it is large enough and put 1, otherwise put 0.

// numerator is above fraction, denominator is below
unsigned divide(unsigned num, unsigned den) {

  unsigned dig = 1;
  unsigned res = 0;

  // shift denominator until it is greater than numerator (overshoot by one even if equal)
  while (den <= num) {
    dig <<= 1;
    den <<= 1;
  }

  // try subtracting each place if possible
  while (dig > 1) {
    dig >>= 1;
    den >>= 1;

    // set result for that digit to 1 if you can perform division
    if (num >= den) {
      num -= den;
      res |= dig;
    }
    // otherwise set bit to 0, this branch doesn't do anything but is here to help you understand
    // else {
    //   res |= 0;
    // }
  }

  return res;
}

int main(int argc, char ** argv) {
  unsigned num = 18;
  unsigned den = 3;
  printf("expected: %d, got: %d\n", num / den, divide(num, den));

}