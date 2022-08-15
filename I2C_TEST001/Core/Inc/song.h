/*
 * song.h
 *
 *  Created on: 2022. 8. 11.
 *      Author: DELL
 */

#ifndef INC_SONG_H_
#define INC_SONG_H_

typedef enum {
	// APB1 == 90[mhz]
	// prescaler == 30-1
	N = 0,
	C = 11762,
	D = 10469,
	E = 9318,
	F = 8791,
	G = 7825,
	A = 6966,
	B = 6200,
} _PITCH;
	extern char song_title_1[] = "1.Traffic Light";
	extern char song_title_2[] = "2.star";


  // Song_Traffic_Light_Note
  extern char *song_note_1[] = {

			"C4N", "C5N", "D5N", "E5N", "C5N","G4N",
			"C5N", "G4N", "C5N", "D5N", "E5N",

			"F3N", "C5N", "D5N", "E5N", "C5N","G4N",
			"C5N", "G4N","C5N","E5N","F5N","E5N","D5N","C5N",

			"C4N", "C5N", "D5N", "E5N", "C5N", "G4N",
			"C5N", "G4N", "C5N", "D5N", "E5N",

			"N5N", "E5N", "F5N", "E5N", "F5N", "E5N", "C5N", "D5N", "N5N",

			"N5N", "C5N", "D5N", "E5N", "C5N","G4N",
			"C5N", "G4N", "C5N", "D5N", "E5N",

			"N5N", "C5N", "D5N", "E5N", "C5N","G4N",
			"C5N", "G4N","C5N","E5N","F5N","E5N","D5N","C5N",

			"N5N", "C5N", "D5N", "E5N", "C5N", "G4N",
			"C5N", "G4N", "C5N", "F5N", "E5N",
			"N5N","E5N","F5N","E5N","F5N","E5N","C5N","D5N","N5N",

			"N5N","D5N","D5N","D5N","D5N", "C5N", "G4N", "C5N", "G4N", "C5N",
			"N5N","C5N","E5N","F5N","E5N","D5N","C5N","D5N",

			"N5N","N5N","C5N","D5N","E5N",
			"N5N","D5N","D5N","D5N","D5N", "C5N", "G4N", "C5N", "G4N", "C5N", "D5N", "E5N",

			"N4N","A4N","A4N","A4N","A4N","G4N","E4N","G4N",
			"N5N","E5N","D5N","C5N","A5N","A5N","G5N",

			"E5N","D5N","C5N","C5N","C5N","D5N","E5N",
			"E5N","D5N","C5N","F5N","F5N","E5N","C5N",

			"N5N","C5N","C5N","E5N","F5N","D5N",
			"E5N","D5N","C5N","A5N","A5N","G5N",

			"E5N","D5N","C5N","C5N","C5N","D5N","E5N",
			"E5N","D5N","C5N","F5N","F5N","E5N","C5N","C5N","E5N","D5N",
			"E5N","F5N","E5N","D5N","C5N",

			"0",
			};

	// Song_Traffic Light_time
  extern int song_time_1[] = {
			4,8,8,4,8,8,
			8,8,8,8,2,

			4,8,8,4,8,8,
			8,8,8,8,8,8,8,8,

			4,8,8,4,8,8,
			8,8,8,8,2,

			4,8,8,8,8,8,8,2,2,

			4,8,8,4,8,8,
			8,8,8,8,2,

			4,8,8,4,8,8,
			8,8,8,8,8,8,8,8,

			4,8,8,4,8,8,
			8,8,8,8,2,
			4,8,8,8,8,8,8,2,2,

			4,8,8,8,8,8,8,8,8,4,
			4,8,8,4,4,8,8,4,

			2,8,8,8,8,
			4,8,8,8,8,8,8,8,8,8,8,2,

			8,8,8,8,8,8,8,8,
			1.5,8,8,4,8,8,4,

			8,8,8,8,8,8,4,
			8,8,4,8,8,8,8,

			8,8,4,8,8,4,
			8,8,4,8,8,4,

			8,8,8,8,8,8,4,
			8,8,4,8,8,8,8,8,8,2,
			8,8,8,8,1,

			1
	};


  // Song_Star_Note
  extern char *song_note_2[] = {

		  "C4N","C4N","G4N","G4N","A4N","A4N","G4N",
		  "F4N","F4N","E4N","E4N","D4N","D4N","C4N",

		  "G4N","G4N","F4N","F4N","E4N","E4N","D4N",
		  "G4N","G4N","F4N","F4N","E4N","E4N","D4N",

		  "C4N","C4N","G4N","G4N","A4N","A4N","G4N",
		  "F4N","F4N","E4N","E4N","D4N","D4N","C4N","0"

  };
  // Song_Star_time
  extern int song_time_2[] = {
		  4,4,4,4,4,4,2,
		  4,4,4,4,4,4,2,

		  4,4,4,4,4,4,2,
		  4,4,4,4,4,4,2,

		  4,4,4,4,4,4,2,
		  4,4,4,4,4,4,2,

		  1
  };
#endif /* INC_SONG_H_ */
