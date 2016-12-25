// junand 25.12.2016

//========================================================================================================================================================

// in HardwareSerial.h buffer size is defined for 16 byte
// in FastLED.cpp dithering is commented

#define SERIAL_BUFFER_SIZE 16
// #define SERIAL_TX_BUFFER_SIZE SERIAL_BUFFER_SIZE
// #define SERIAL_RX_BUFFER_SIZE SERIAL_BUFFER_SIZE

#define NO_DITHERING 1

#include <Arduino.h>

#include <avr/pgmspace.h>
#include <Streaming.h>

#include <Tlc5940.h>

#include <FastLED.h>

#define VERBOSE 0
#define COMMAND_PERFIX "###"

//========================================================================================================================================================

const static int NUM_LEDS = 15;

static CRGB leds[NUM_LEDS];
static CRGB color_rgb = CRGB::Red;
static uint8_t hue = HUE_RED;
static uint8_t red = 0;
static uint8_t green = 0;
static uint8_t blue = 0;

static int mode = 5;
static int delta = 4;
static int update_per_seconds = 50;
static boolean showProcessed = false;
static uint8_t brightness = 20;

static int pos = -1;

//The ASCII buffer for recieving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

#define BAUDRATE 115200

//========================================================================================================================================================

// template <uint8_t DATA_PIN, uint8_t CLOCK_PIN, EOrder RGB_ORDER = RGB, uint8_t SPI_SPEED = DATA_RATE_MHZ(10)>
template <EOrder RGB_ORDER = RGB> class TLC5940Controller : public CPixelLEDController<RGB_ORDER> {
	
	// inline void writeLed ( uint8_t led, uint8_t r, uint8_t g, uint8_t b ) __attribute__((always_inline)) {
	void writeLed ( uint8_t led, uint8_t r, uint8_t g, uint8_t b ) {
		
		int chip = led / 5;
		int chipled = led % 5;
		
		// Serial << "led=" << led << " chip=" << chip << " chipled=" << chipled << " r=" << r << " g=" << g << " b=" << b;
		
		const uint16_t tlc_scale = 16;
		uint16_t red16 = tlc_scale * r;
		uint16_t green16 = tlc_scale * g;
		uint16_t blue16 = tlc_scale * b;
		
		// Serial << " red16=" << red16 << " green16=" << green16 << " blue16=" << blue16 << endl;

		Tlc.set ( 16 * chip + 3 * chipled + 0, red16 ); // hint: 4096 / 256 = 16; max grayscale of tlc5940 divided by max rgb
		Tlc.set ( 16 * chip + 3 * chipled + 1, green16 ); // hint: 4096 / 256 = 16; max grayscale of tlc5940 divided by max rgb
		Tlc.set ( 16 * chip + 3 * chipled + 2, blue16 ); // hint: 4096 / 256 = 16; max grayscale of tlc5940 divided by max rgb
		
	}

public:
	TLC5940Controller () {}

	virtual void init () {
		Tlc.init ( 0 );
	}
	
protected:

	virtual void showPixels ( PixelController<RGB_ORDER>& pixels ) {
		
		int led = 0;
		while ( pixels.has ( 1 ) ) {
			
			uint8_t r = pixels.loadAndScale0 ();
			uint8_t g = pixels.loadAndScale1 ();
			uint8_t b = pixels.loadAndScale2 ();
			
			// if ( VERBOSE ) Serial << "r=" << r << " g=" << g << " b=" << b << endl;

			writeLed ( led++, r, g, b );
			pixels.advanceData ();
			// pixels.stepDithering ();
		}
		
		// Tlc.update() sends the data to the TLCs.  This is when the LEDs will actually change.
		Tlc.update ();
		
	}

};

//========================================================================================================================================================

static TLC5940Controller<RGB> controller = TLC5940Controller<RGB> ();

void setup () {
    
    delay ( 1000 ); // power-up safety delay

    Serial.begin ( BAUDRATE );
	
	if ( VERBOSE ) Serial << "---------------------" << endl;
	if ( VERBOSE ) Serial << "tx_buffer_size=" << SERIAL_TX_BUFFER_SIZE << " rx_buffer_size" << SERIAL_RX_BUFFER_SIZE << endl;
	if ( VERBOSE ) Serial << "Number of TLC5940: " << NUM_TLCS << endl;
    
	FastLED.addLeds ( &controller, leds, NUM_LEDS, 0 );
    FastLED.setBrightness ( brightness );
	FastLED.setDither ( DISABLE_DITHER );

    if ( VERBOSE ) Serial << "setup beendet" << endl;
}

//========================================================================================================================================================

void show () {

    /* Tlc.clear() sets all the grayscale values to zero, but does not send
    them to the TLCs.  To actually send the data, call Tlc.update() */
    Tlc.clear ();

	for ( int led = 0; led < NUM_LEDS; led++ ) {
		
        /* Tlc.set(channel (0-15), value (0-4095)) sets the grayscale value for
        one channel (15 is OUT15 on the first TLC, if multiple TLCs are daisy-
        chained, then channel = 16 would be OUT0 of the second TLC, etc.).

        value goes from off (0) to always on (4095).

        Like Tlc.clear(), this function only sets up the data, Tlc.update()
        will send the data. */

		// only 15 channels of a chip are used; last channels of all three chips are joined as led 16, but not used 
		int chip = led / 5;
		int chipled = led % 5;
		for ( int i = 0; i < 3; i++ ) { // iterate over rgb
			Tlc.set ( 16 * chip + 3 * chipled + i, leds [led][i] * 16 ); // hint: 4096 / 256 = 16; max grayscale of tlc5940 divided by max rgb
		}
		
	}

    // Tlc.update() sends the data to the TLCs.  This is when the LEDs will actually change.
    Tlc.update ();

    delay ( 1 );

}

//========================================================================================================================================================

void loop () {

    uint8_t i;

    if ( buflen < (BUFSIZE - 1) ) get_command ();
	// Serial << "BUFLEN=" << buflen << endl;
    if ( buflen >  0 ) {
		if ( VERBOSE ) Serial << "buflen=" << buflen << endl;
        process_commands ();
        buflen = buflen - 1;
        bufindr = (bufindr + 1) % BUFSIZE;
    }
    
    if ( mode && !showProcessed ) {
		// Serial << "BLACK 111" << endl;
        fill_solid ( leds, NUM_LEDS, CRGB::Black );
        FastLED.setBrightness (  brightness );
		// Serial << "BLACK 222" << endl;
    }

    switch ( mode ) {

        // same color for all leds, but color is continously changing
        // M01 Hnnn Unnn Lnnn; ohne H wird rot genommen
        case 1: 
            if ( ++hue > 255 ) hue = 0;
            color_rgb.setHue ( hue );
            fill_solid ( leds, NUM_LEDS, color_rgb );
            break;

        // random led, only one led at a time, default color is white
        // M02 Hnnn - ohne H wird weiß genommen
        case 2: 
            i = random8 ( NUM_LEDS );
            leds [i] = color_rgb;
            break;
            
        // random color at random led
        // M03 Unnn Lnnn - all random
        case 3: 
            i = random8 ( NUM_LEDS );
            leds [i].setRGB ( random8 (), random8 (), random8 () );
            break;
            
        // rainbow 
        // M04 Hnnn Dnnn Unnn Lnnn; rainbow
        case 4: 
            hue += delta;
            if ( hue > 255 ) hue -= 256;
            fill_rainbow ( leds, NUM_LEDS, hue, delta );
            break;
            
        // all leds with last color
        // M05 Hnnn Lnnn
        case 5: 
            color_rgb.setHue ( hue );
            fill_solid ( leds, NUM_LEDS, color_rgb );
            update_per_seconds = 1;
            break;
            
        // one color for all leds in rgb
        // M06 Rnnn Gnnn Bnnn Lnnn
        case 6: 
            color_rgb.setRGB ( red, green, blue );
			if ( pos == -1 ) { 
				fill_solid ( leds, NUM_LEDS, color_rgb );
			}
			else {
				leds [pos] = color_rgb;
			}
            break;
            
        default:
            break;
            
    }

    if ( mode ) {
		if ( update_per_seconds > 0 ) {
			FastLED.delay ( 1000 / update_per_seconds ); // calls implicitly show ()
		}
		else {
			if ( !showProcessed ) {
				if ( VERBOSE ) Serial << "static show" << endl;
				FastLED.show ();
				if ( VERBOSE ) Serial << "after show" << endl;
				showProcessed = true;
			}
		}
    }

}

//========================================================================================================================================================

void get_command () {
  
  while ( Serial.available () > 0  && buflen < BUFSIZE ) {
    
    serial_char = Serial.read ();
	
	if ( VERBOSE ) Serial << "c=" << serial_char << " available=" << Serial.available () << endl;

    if ( serial_char == '\n' ||
         serial_char == '\r' ||
         (serial_char == ':' && comment_mode == false) ||
         serial_count >= (MAX_CMD_SIZE - 1) 
    ) {
		
      if ( !serial_count ) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      
      cmdbuffer [bufindw][serial_count] = 0; //terminate string
	  if ( VERBOSE ) Serial << "line end serial_count=" << serial_count << " cmdbuffer=]" << cmdbuffer [bufindw] << "[" << endl;
      
      if ( !comment_mode ) {
        bufindw = (bufindw + 1) % BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer

    }
    else {
      
      if ( serial_char == ';' ) comment_mode = true;
      if ( !comment_mode ) cmdbuffer [bufindw][serial_count++] = serial_char;
      
    }

  }

}

uint8_t code_value () {
  
  return strtol ( &cmdbuffer [bufindr][strchr_pointer - cmdbuffer [bufindr] + 1], NULL, 10 );

}

bool code_seen ( char code ) {
  
  strchr_pointer = strchr ( cmdbuffer [bufindr], code );
  return strchr_pointer != NULL;  //Return True if a character was found
  
}

void process_commands () {
	
	boolean command_processed = false;
	
#ifdef COMMAND_PERFIX	
	boolean command_seen = strlen ( cmdbuffer [bufindr] ) > 3 && strncmp ( COMMAND_PERFIX, cmdbuffer [bufindr], 3 ) == 0;
	if ( !command_seen ) {
		if ( VERBOSE ) Serial << "no command=" << cmdbuffer [bufindr] << endl;	
		return;
	}
#endif	

	// if ( code_seen ( '[' ) ) {
	// }
    // else if ( code_seen ( 'X' ) ) {
        // softwareReset ();
    // }   
    if ( code_seen ( 'M' ) ) {
		
		command_processed = true;
		
		int code = (int) code_value ();
		// if ( VERBOSE ) Serial << "M with code=" << code << endl;
		
        switch ( code ) {

			// same color for all leds
            case 1: // M01 Hnnn Lnnn; ohne H wird rot genommen
                mode = 1;
                hue = HUE_RED;
                update_per_seconds = 10;
                if ( code_seen ( 'H' ) ) {
                    hue = code_value ();
                }
                if ( code_seen ( 'L' ) ) {
                    brightness = code_value ();
                }
                break;

			// random led, only one led at a time, default color is white
            case 2: // M02 Hnnn Unnn Lnnn - ohne H wird weiß genommen
                mode = 2;
                color_rgb = CRGB::White;
                if ( code_seen ( 'H' ) ) {
                    hue = code_value ();
					color_rgb.setHue ( hue );
                }
                update_per_seconds = 10;
                if ( code_seen ( 'U' ) ) {
                    update_per_seconds = code_value ();
                }
                if ( code_seen ( 'L' ) ) {
                    brightness = code_value ();
                }
                break;

			// random color at random led
            case 3: // M03 Unnn Lnnn - all random
                mode = 3;
                update_per_seconds = 10;
                if ( code_seen ( 'U' ) ) {
                    update_per_seconds = code_value ();
                }
                if ( code_seen ( 'L' ) ) {
                    brightness = code_value ();
                }
                break;

			// rainbow 
            case 4: // M04 Hnnn Dnnn Unnn Lnnn; rainbow
                mode = 4;
                hue = HUE_RED;
                if ( code_seen ( 'H' ) ) {
                    hue = code_value ();
                }
                delta = 5;
                if ( code_seen ( 'D' ) ) {
                    delta = code_value ();
                }
                update_per_seconds = 10;
                if ( code_seen ( 'U' ) ) {
                    update_per_seconds = code_value ();
                }
                if ( code_seen ( 'L' ) ) {
                    brightness = code_value ();
                }
                break;

            case 5: // M05 Hnnn
                mode = 5;
                hue = HUE_RED;
                if ( code_seen ( 'H' ) ) {
                    hue = code_value ();
                }
                update_per_seconds = 0; // static
				showProcessed = false;
                if ( code_seen ( 'L' ) ) {
                    brightness = code_value ();
                }
				if ( VERBOSE ) Serial << "M05: hue=" << hue << " brightness=" << brightness << endl;
                break;
				
			case 6: // M06 Rnnn Gnnn Bnnn Lnnn
                mode = 6;
                red = 0; // default black
				green = 0;
				blue = 0;
                update_per_seconds = 0; // static
				showProcessed = false;
                if ( code_seen ( 'R' ) ) {
                    red = code_value ();
					if ( VERBOSE ) Serial << "red=" << red << endl;
                }
                if ( code_seen ( 'G' ) ) {
                    green = code_value ();
					if ( VERBOSE ) Serial << "green=" << green << endl;
                }
                if ( code_seen ( 'B' ) ) {
                    blue = code_value ();
					if ( VERBOSE ) Serial << "blue=" << blue << endl;
                }
                if ( code_seen ( 'L' ) ) {
                    brightness = code_value ();
					if ( VERBOSE ) Serial << "light=" << brightness << endl;
                }
				pos = -1;
                if ( code_seen ( 'P' ) ) {
                    pos = code_value ();
					if ( pos >= NUM_LEDS ) pos = NUM_LEDS - 1;
                }
                break;

            default:
				command_processed = false;
				break;

        }

    }
    else if ( code_seen ( 'U' ) ) {
        update_per_seconds = code_value ();
		command_processed = true;
    }
    else if ( code_seen ( 'L' ) ) {
        brightness = code_value ();
		showProcessed = false;
		command_processed = true;
    }
    else if ( code_seen ( 'H' ) ) {
        hue = code_value ();
		command_processed = true;
    }
    else if ( code_seen ( 'D' ) ) {
        delta = code_value ();
		command_processed = true;
    }
    else if ( code_seen ( 'I' ) ) {
        // display some vars
        Serial << "INFO: mode=" << mode << " updates=" << update_per_seconds << " delta=" << delta << " brightness=" << brightness;
		Serial << " hue=" << hue << " red=" << red << " green=" << green << " blue" << blue << endl;
		command_processed = true;
    }
	else if ( code_seen ( 'P' ) ) {
		pos = code_value ();
		if ( pos >= NUM_LEDS ) pos = NUM_LEDS - 1;
		showProcessed = false;
		command_processed = true;
	}
    else {
        if ( VERBOSE ) Serial << "MSG_UNKNOWN_COMMAND \"" << cmdbuffer [bufindr] << "\"" << endl;
    }

	if ( command_processed ) Serial.print ( "ok\n" );
  
}

//========================================================================================================================================================



