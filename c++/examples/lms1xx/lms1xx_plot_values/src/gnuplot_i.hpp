////////////////////////////////////////////
//
// A C++ interface to gnuplot. 
//
// This is a direct translation from the C interface
// written by N. Devillard (which is available from
// http://ndevilla.free.fr/gnuplot/).
//
// As in the C interface this uses pipes and so wont
// run on a system that does'nt have POSIX pipe 
// support
//
// Rajarshi Guha
// <rajarshi@presidency.com>
//
// 07/03/03
//
// 26/01/04 - Gnuplot::cmd() was rewritten to accept a
// char* rather than a std::string, thus avoiding the
// va_start warning at compile time
// /////////////////////////////////////////

#ifndef _GNUPLOT_PIPES_H_
#define _GNUPLOT_PIPES_H_

#include <stdarg.h>
#include <unistd.h>

#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <list>
#include <vector>
#include <stdexcept>

#define GP_MAX_TMP_FILES    64
#define GP_TMP_NAME_SIZE    512
#define GP_CMD_SIZE         1024
#define GP_TITLE_SIZE       80

using namespace std;

class GnuplotException : public runtime_error
{
public:
  GnuplotException(const string &msg) : runtime_error(msg){}
};

class Gnuplot
{
private:
  FILE            *gnucmd;
  string           pstyle;
  vector<string>   to_delete;
  int              nplots;
  bool             get_program_path(const string);
  bool             valid;
public:
  Gnuplot();

  // set a style during construction
  Gnuplot(const string &);
        
  // The equivilant of gnuplot_plot_once, the two forms
  // allow you to plot either (x,y) pairs or just a single
  // vector at one go
  Gnuplot(const string &, // title
	  const string &, // style
	  const string &, // xlabel
	  const string &, // ylabel
	  vector<double>, vector<double>);
        
  Gnuplot(const string &, //title
	  const string &, //style
	  const string &, //xlabel
	  const string &, //ylabel
	  vector<double>);
        
  ~Gnuplot();

  // send a command to gnuplot
  void cmd(const char*, ...);

  // set line style
  void set_style(const string &);

  // set y and x axis labels
  void set_ylabel(const string &);
  void set_xlabel(const string &);
  void set_zlabel(const string &);
  
  // set axis - ranges
  void set_xrange(const int iFrom, const int iTo);
  void set_yrange(const int iFrom, const int iTo);  
  void set_zrange(const int iFrom, const int iTo);  
  
  // set palette range
  void set_cbrange(const int iFrom, const int iTo);
  
  
  // plot a single vector
  void plot_x(vector<double>, 
	      const string & // title
	      );

  // plot x,y pairs
  void plot_xy(vector<double>, vector<double>, 
	       const string  & // title
	       );

  // plot an equation of the form: y = ax + b
  // You supply a and b
  void plot_slope(
		  double, // a
		  double, // b 
		  const string & // title
		  );

  // plot an equation supplied as a string
  void plot_equation(
		     const string &, // equation 
		     const string &  // title
		     );

  // if multiple plots are present it will clear the plot area
  void reset_plot(void);

  bool is_valid(void);

  void plot_xyz(vector<double>, vector<double>, vector<double>, const string  & );
  
  void plot_image(unsigned char * ucPicBuf, int iWidth, int iHeight, const string &title);
        
};

#endif
