#include <Arduino.h>
#include <lvgl.h>
#include <demos/lv_demos.h>

// I2C touch controller FT6336U
#if defined(TFT)
// FOR I2C display touch sensor
#include <classFT6336U.h>
// FOR SPI Display
#include <TFT_eSPI.h>

#define I2C_SDA 18
#define I2C_SCL 19
#define INT_N_PIN 39

// LCD backlight control
// TFT_BL GPIO pin defined in user_setup.h of tft_eSPI
// setting PWM properties
#define BL_PWM_FREQ 5000
#define BL_PWM_CHANNEL 0
#define BL_PWM_RESOLUTION 8

// TFT instance
TFT_eSPI tft = TFT_eSPI(TFT_WIDTH, TFT_HEIGHT);
// touch controller instance
classFT6336U ft6336u = classFT6336U(I2C_SDA, I2C_SCL, INT_N_PIN);
#endif

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ TFT_WIDTH * 10 ];
lv_indev_t *myInputDevice;

#if defined(GFX)
#define LGFX_USE_V1         // set to use new version of library
#include <LovyanGFX.hpp>    // main library

class LGFX : public lgfx::LGFX_Device
{

  lgfx::Panel_ST7796  _panel_instance;  // ST7796UI
  lgfx::Bus_Parallel8 _bus_instance;    // MCU8080 8B
  lgfx::Light_PWM     _light_instance;
  lgfx::Touch_FT5x06  _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.freq_write = 20000000;    
      cfg.pin_wr = 47;             
      cfg.pin_rd = -1;             
      cfg.pin_rs = 0;              

      // LCD data interface, 8bit MCU (8080)
      cfg.pin_d0 = 9;              
      cfg.pin_d1 = 46;             
      cfg.pin_d2 = 3;              
      cfg.pin_d3 = 8;              
      cfg.pin_d4 = 18;             
      cfg.pin_d5 = 17;             
      cfg.pin_d6 = 16;             
      cfg.pin_d7 = 15;             

      _bus_instance.config(cfg);   
      _panel_instance.setBus(&_bus_instance);      
    }

    { 
      auto cfg = _panel_instance.config();    

      cfg.pin_cs           =    -1;  
      cfg.pin_rst          =    4;  
      cfg.pin_busy         =    -1; 

      cfg.panel_width      =   320;
      cfg.panel_height     =   480;
      cfg.offset_x         =     0;
      cfg.offset_y         =     0;
      cfg.offset_rotation  =     0;
      cfg.dummy_read_pixel =     8;
      cfg.dummy_read_bits  =     1;
      cfg.readable         =  true;
      cfg.invert           = true;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       =  true;

      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();    

      cfg.pin_bl = 45;              
      cfg.invert = false;           
      cfg.freq   = 44100;           
      cfg.pwm_channel = 7;          

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);  
    }

    { 
      auto cfg = _touch_instance.config();

      cfg.x_min      = 0;
      cfg.x_max      = 319;
      cfg.y_min      = 0;  
      cfg.y_max      = 479;
      cfg.pin_int    = 7;  
      cfg.bus_shared = true; 
      cfg.offset_rotation = 0;

      cfg.i2c_port = 1;
      cfg.i2c_addr = 0x38;
      cfg.pin_sda  = 6;   
      cfg.pin_scl  = 5;   
      cfg.freq = 400000;  

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);  
    }
    setPanel(&_panel_instance); 
  }
};

static LGFX tft;            // declare display variable

// Variables for touch x,y
// static int32_t x,y;
#endif

#if defined(TFT)
#if LV_USE_LOG != 0
// Serial debugging if enabled
void my_print(const char *buf)
{
  Serial.printf(buf);
  Serial.flush();
}
#endif
#endif

uint32_t chipId = 0;

#if defined(GFX)
/*
    lcd interface
    transfer pixel data range to lcd
*/
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  int w = (area->x2 - area->x1 + 1);
  int h = (area->y2 - area->y1 + 1);

  tft.startWrite(); /* Start new TFT transaction */
  tft.setAddrWindow(area->x1, area->y1, w, h); /* set the working window */

  tft.writePixels((lgfx::rgb565_t*)&color_p->full, w*h);

  tft.endWrite(); /* terminate TFT transaction */
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

/*
    touch pad interface
    test for touch and report RELEASED / or PRESSED + x/y back
*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
   uint16_t touchX, touchY;

   bool touched =tft.getTouch(&touchX, &touchY);

   if(!touched)
   {
      data->state = LV_INDEV_STATE_REL;
   }
   else
   {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touchX;
      data->point.y = touchY;

      Serial.print( "Data x " );
      Serial.println( touchX );

      Serial.print( "Data y " );
      Serial.println( touchY );
   }
}
#endif


#if defined(TFT)
/*
    lcd interface
    transfer pixel data range to lcd
*/
void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);

  lv_disp_flush_ready(disp);
}

/*
    touch pad interface
    test for touch and report RELEASED / or PRESSED + x/y back
*/
void my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
  typePoint ts;
  bool touched = ft6336u.readTouchPoint(&ts);

  // no touch detected
  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
    return;
  }
  // get coordinates and write into point structure
  data->point.x = ts.x;
  data->point.y = ts.y;
  data->state = LV_INDEV_STATE_PR;

// #if defined(DEBUG_TOUCH)
  Serial.print(F("[tp32] touch data (x,y): "));
  Serial.print(data->point.x);
  Serial.print(F(","));
  Serial.println(data->point.y);
// #endif
}
#endif

void setup() {
  Serial.begin(115200);

  #if defined(GFX)
  lv_init();
  // lv_img_cache_set_size(10);
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); // register print function for debugging
#endif

  tft.init();
  tft.fillScreen(TFT_BLACK);

  // initialise draw buffer
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, TFT_WIDTH * 10);

  // initialise the display
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  // settings for display driver
  disp_drv.hor_res = TFT_WIDTH;
  disp_drv.ver_res = TFT_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // initialise the input device driver (touch panel)
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  // set timings for LongPress ,RepeatTime and gesture detect
  indev_drv.long_press_time = 500;
  indev_drv.long_press_repeat_time = 200;
  indev_drv.gesture_limit = 40;
  indev_drv.scroll_limit = 3;

  lv_demo_widgets(); 
  #endif

#if defined(TFT)
  // set up for backlight dimming (PWM)
  ledcSetup(BL_PWM_CHANNEL, BL_PWM_FREQ, BL_PWM_RESOLUTION);
  ledcAttachPin(TFT_BL, BL_PWM_CHANNEL);
  ledcWrite(BL_PWM_CHANNEL, 127);

  lv_init();
  lv_img_cache_set_size(10);
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); // register print function for debugging
#endif

  // Initialise the TFT screen
  tft.init();
  tft.fillScreen(TFT_BLACK);

  // touch pad
  ft6336u.begin();
  pinMode(39, INPUT);

  // initialise draw buffer
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, TFT_WIDTH * 10);
  // initialise the display
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  // settings for display driver
  disp_drv.hor_res = TFT_WIDTH;
  disp_drv.ver_res = TFT_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // initialise the input device driver (touch panel)
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  myInputDevice = lv_indev_drv_register(&indev_drv);
  // set timings for LongPress ,RepeatTime and gesture detect
  indev_drv.long_press_time = 500;
  indev_drv.long_press_repeat_time = 200;
  indev_drv.gesture_limit = 40;
  indev_drv.scroll_limit = 3;

  lv_demo_widgets(); 
#endif

  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}

	Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
	Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId);

  Serial.printf("flash Size: %d \n", ESP.getFlashChipSize());
  Serial.printf("Psram Size: %d \n", ESP.getPsramSize());
  Serial.printf("ram Size: %d \n", ESP.getHeapSize());

}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  delay( 5 );
}