#ifndef __FLOATPRINT_H__
#define __FLOATPRINT_H__

template <size_t bufsize> class FloatPrint : public Print
{
public:
  FloatPrint(void) {}
  using Print::write;
  virtual size_t write(uint8_t c) { if (pos < bufsize) buf[pos++] = c; return 1; }

  void print(Print &p, double n, uint8_t width, uint8_t prec)
  {
    pos = 0;
    Print::print(n, prec);
    
    uint8_t r = width - pos;
    while (r--) p.write(' ');
    r = 0;
    while (r < pos) p.write(buf[r++]);
  }

  void println(Print &p, double n, uint8_t width, uint8_t prec)
  {
    print(p, n, width, prec);
    p.println();
  }

private:
  char buf[bufsize];
  uint8_t pos;
};

#endif