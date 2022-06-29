#pragma once

#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include <vector>
#include <string>

namespace ctre {
	namespace phoenix {
		namespace music {

			/**
			 * An Orchestra is used to play music through Talon FX motor controllers.
			 * It uses a "Chirp" (.chrp) music file that can be generated using Phoenix Tuner.
			 * 
			 * Chirp files are generated from standard MIDI files.
			 * Each Talon FX can only play a single track within the music file.
			 * For multi-track files, multiple Talon FXs are needed.
			 *  ie, The first track will be played through the first Talon FX added,
			 *  the second track will be played through the second Talon FX added, etc.
			 * 
			 * Any Chirp file located in the src/main/deploy directory of your FRC project 
			 *  will automatically be copied to the roboRIO on code deploy.
			 * 
			 * To use the Orchestra:
			 *  - Add the Talon FXs to be used as instruments
			 *  - Load the Chirp file to be played using the LoadMusic routine.
			 * 
			 * Once ready, the Orchestra can be controlled using standard
			 * play/pause/stop routines.
			 * 
			 * New music files can be loaded at any time.
			 * 
			 * The robot must be enabled to play music.
			 * 
			 * Calling set on any of the TalonFX instruments while the orchestra is
			 * playing will pause the orchestra.
			 */
			class Orchestra {
			private:
				void* m_handle;
			public:
				/**
				 * Constructor for an Orchestra Object.
				 * Call AddInstrument after this to add the instruments.
				 */
				Orchestra();
				/**
				 * Loads a Chirp file at the specified file path.
				 * 
				 * If the Chirp file is inside your "src/main/deploy" directory
				 * this file will be automatically deployed to a default directory in
				 * the RoboRIO when you deploy code. For these files, the name and file 
				 * extension is sufficient.
				 * 
				 * Use Tuner to create a Chirp file.
				 * @param filepath
				 * 		    The path to the Chirp File.
				 * @return Error Code generated by function. 0 indicates no error. 
				 */
				ErrorCode LoadMusic(const std::string& filePath);
				/**
				 * Plays the music file that's loaded. 
				 * If the player is paused, this will resume.
				 * This will also resume a song if the orchestra was interrupted.
				 * @return Error Code generated by function. 0 indicates no error. 
				 */
				ErrorCode Play();
				/**
				 * Pauses the music file that's loaded. 
				 * This saves the current position in the track, so it can be resumed later.
				 * Pausing while stopped is an invalid request.
				 * @return Error Code generated by function. 0 indicates no error. 
				 */
				ErrorCode Pause();
				/**
				 * Stops the music file that's loaded. 
				 * This resets the current position in the track to the start.
				 * @return Error Code generated by function. 0 indicates no error. 
				 */
				ErrorCode Stop();
				/**
				 * Returns whether the current track is actively playing or not
				 * @return True if playing, false otherwise 
				 */
				bool IsPlaying();
				/**
				 * @return The current timestamp of the music file (playing or paused) in milliseconds.
				 * The timestamp will reset to zero whenever LoadMusic() or Stop() is called.
				 * If IsPlaying() returns false, this routine can be used to determine if music is stopped or paused.
				 */
				uint32_t GetCurrentTime();
				/**
				 * Adds another instrument to the orchestra.
				 * @param instrument
				 * 		    TalonFX to add to orchestra
				 * @return Error Code generated by function. 0 indicates no error. 
				 */
				ErrorCode AddInstrument(ctre::phoenix::motorcontrol::can::TalonFX& instrument);
				/**
				 * Clears all instruments in the orchestra.
				 * @return Error Code generated by function. 0 indicates no error. 
				 */
				ErrorCode ClearInstruments();
			};
		}
	}
}
