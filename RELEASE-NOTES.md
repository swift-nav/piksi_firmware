Piksi Firmware Release Notes
============================

Contents
--------
 * [v0.21](#v0.21)
 * [v0.20](#v0.20)
 * [v0.19](#v0.19)
 * [v0.18](#v0.18)
 * [v0.17](#v0.17)
 * [v0.16](#v0.16)
 * [v0.15](#v0.15)
 * [v0.14](#v0.14)
 * [v0.13](#v0.13)
 * [v0.12](#v0.12)
 * [v0.11](#v0.11)
 * [v0.10](#v0.10)
 * [v0.8](#v0.8)

v0.21 <a name="v0.21"></a>
-----

#### Core
 * Fixes "dgnss_baseline returned error: -2" bug when dropping a satellite.
 * Reduce occurence of "status glitches" that were observed by users
   integrating Piksi with Pixhawk by increasing RTK availablity.
 * 1PPS output on debug connector (pin DEBUG1)
 * New tracking lock detection architecture
 * Allow NMEA rates to be set to zero to disable
 * Reduce default baud and air rates for 3DR radios
 * Reduce observation message size for Mavlink encapsulation
 * Reduce serial bus traffic to improve console responsiveness on user machines
 * Fix uninitialized week number in SBP ephemeris TOC
 * Numerous minor fixes and improvements

v0.20 <a name="v0.20"></a>
-----
#### Core
 * Updates to libsbp for stability
  * Deprecate MSG_PRINT in favor of MSG_LOG which includes log level in protocol
  * Remove strings that indicate log levels from msg print prefixes
  * Update the following messages in order to widen PRN field and rename to SID
    * SBP_MSG_TRACKING
    * SBP_MSG_ACQ_RESULT
    * SBP_MSG_OBS
    * SBP_MSG_EPHEMERIS
    * SBP_MSG_TRACKING_STATE
 * Add sanity check for user entered base station position upon broadcast of location
 * Change tracking threshold to 31 db-hz to help maintain track
 * Fix known bug where Piksi could operate incorrectly during GPS week rollover

v0.19 <a name="v0.19"></a>
-----

#### Core

 * Fix bug where acquisition could get stuck searching an empty Doppler range
 * Fix buffer overflows when building NMEA messages
 * Add better checks and thread safety to random number generator
 * Add GPRMC, GPVTG, and GPGLL NMEA messages
 * Settings and FileIO messages are now directional

#### libswiftnav

 * Receiver Autonomous Integrity Monitoring (RAIM) to filter out
   bad position solutions
 * Various off by 1 fixes
 * Add phase lock counter to local observations
 * Improved PLL lock detection
 * Change tracking C/N0 estimator to dQ/I
 * More robust nav message decoding
 * Update UTC time offset for leap second on June 30th, 2015

v0.18 <a name="v0.18"></a>
-----

#### Core

 * Fixed flash corruption bug - increases % of time device reaches the
   correct Fixed RTK solution and reduces device crashes
 * Fixed bug in acquisition core in SwiftNAP that caused device crashes

#### libswiftnav

 * Float filter robustness improvements
 * Refactor of baseline / differential positioning management code

#### libsbp

 * Ephemeris and bootloader handshake messages have been changed and the
   old versions deprecated
 * See [https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf](https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf)
   for documentation on current version of SBP messages
 * Numerous changes to other messages, which are supported by piksi_tools

v0.17 <a name="v0.17"></a>
-----

 * More robust nav bit / Ephemeris decoding - reduces number of position
   outliers and Fixed to Float transitions
 * Improve C/N0 estimation filter and reduce tracking integration length -
   reduces mechanical flexing / heat sensitivity
 * Use a larger initial covariance in Kalman filter - decreases chance of
   an incorrect Fixed point solution
 * Uses highest elevation satellite for nominal pseudorange
 * More robust checks on tracking channel measurements before using
   in PVT solution

v0.16 <a name="v0.16"></a>
-----

#### Core

 * Acquisition / Tracking
   * Increased sensitivity: correlate for 20ms intervals (rather than 1ms) after nav bit sync
   * Avoid bad ephemerides by treating new ephemerides as candidates until received a second time
   * Reset carrier phase ambiguity for a tracking channel if its SNR falls below a threshold
   * Prioritize satellites the base station is tracking in the acquisition management
   * Now using true C/N0 SNR estimator
 * Fault Handling
   * Watchdog and hard fault handling
 * External Event Capture
   * Add external event timing input to DEBUG header (documentation for this feature coming soon)
 * Miscellaneous
   * Use external antenna as default rather than gated depending on external antenna current draw

#### Piksi Console / Scripts

 * Python scripts (Piksi Console, bootload script, etc) have been moved to https://github.com/swift-nav/piksi_tools
 * Remove Python / Piksi Console dependencies from toolchain provisioning

v0.15 <a name="v0.15"></a>
-----

 * Fixes to setup script for new and untracked dependencies

#### Core

 * Fixes to libswiftnav to improve firmware stability
   * Fix for known crash when receiver drops from 5->4 satellites
   * Fix "LD Factorization" message  & bug
   * Fix for suddden large baseline bug when dropping & reacquiring sats
 * Send RTK absolute positions as outputs over the MSG_POS_LLH (id 0x0201) and
   MSG_POS_ECEF (id 0x0200) SBP messages - (Please review the SBP documentation
   of the flags field for both messages)
    * A flag of 0 represents single point position
    * A flag of 1 represents the float solution
    * A flag of 2 represents the fixed RTK solution
 * Add log level prefixes to messages prints (ERROR, WARN, INFO & DEBUG)
 * Remove SBP functionality from libswiftnav and use libsbp instead to handle
   communication
 * Piksi's messages, defined by Swift Binary Protocol (SBP), are now more fully
   documented at http://github.com/swift-nav/libsbp. See the libsbp README for
   more details.

#### Console / serial_link.py

 * Add display of psuedo absolute positions on solution tab
 * Annotate settings to provide a description and notes for each setting
 * Add Json serialization option to serial link functionality
 * Provide a progress indicator for firmware updates
 * Add a pause and clear button to the console log
 * Add Piksi serial number to the console window header
 * Use libsbp to decode messages from Piksi
 * Watchdog timer option on Piksi heartbeat messages
 * If running console from the source, update dependencies or rerun the setup.sh
   script to grab the necessary dependencies

v0.14 <a name="v0.14"></a>
-----

 * Numerous fixes to setup script for Ubuntu, OS X, ARM and RasberryPi targets

#### Core

 * Make low latency mode the default
 * Added chi-squared test on baseline solution residual to catch a large class
   of potential errors, including some cycle slips and incorrect IAR
   initializations
 * Handle the `DMA RX buffer overrun` and `observation message packing: integer
   overflow` errors more gracefully, these are no longer a critical errors and
   the system will continue to operate correctly
 * Fixes a bug that periodically caused a `Obs Matching: t_base < t_rover`
   error message and prevented any `BASELINE` messages from being generated
 * Improvements to integer ambiguity resolution speed when adding new
   satellites
 * New tracking loop filter architecture

#### Console / serial_link.py

 * Added options to log SBP data to a file in the console and serial_link.py
 * Ephemerides are now saved alongside observation files from console
 * serial_link.py now has options to reset the device upon connection and to
   exit after a timeout has elapsed
 * More compact layout of GUI elements

v0.13 <a name="v0.13"></a>
-----

 * Fix a bug resulting in a stack overflow when satellite set changes
 * Fix a bug in the NMEA output when the longitude magnitude was >127
 * Simulator mode enable is now a setting rather than a button in the console
 * Numerous improvements to the development environment setup script including
   now using Ansible to provision the required tools

v0.12 <a name="v0.12"></a>
-----

 * Fixes a bug where a "disorder in the sdiffs" message is displayed and the
   RTK solution output stops
 * Fixes an issue where Piksi would stop outputting observations and solutions
   for a period of several minutes before resuming
 * Adds a "lock count" field to the observation message format, allowing the
   remote receiver to detect when a satellite has been reinitialized. This
   fixes a problem where the RTK solution would become large and erronious when
   one receiver is restarted or satellites are rapidly lost and re-gained
 * Fixes issue where console would freeze on start-up
 * Increases the precision of settings values, fixing an issue where the base
   station position would lose precision if the settings were saved and the
   Piksi was then reset
 * Added a "Reset to factory defaults" button to the console
 * Fixes the center on solution button in the console
 * Fixes the start date in RINEX file headers
 * Improves handling of the Piksi being unplugged in the console

v0.11 <a name="v0.11"></a>
-----

 * Add ‘low latency mode’ which provides very low latency baseline outputs
   irrespective of the radio communications delay with a small accuracy penalty
 * Fix a stack overflow bug that occurs when the number of satellites in common
   falls below 5
 * Improve the robustness of the firmware update procedure

v0.10 <a name="v0.10"></a>
-----

#### Core

 * FFT based acquisition core for ~100x increase in acquisition speed
 * Increased from 9 to 11 tracking channels
 * Long SPI transfers to the NAP now use DMA
 * Numerous improvements to the simulator to make it more closely match the
   non-simulated behaviour
 * Fix bug where settings were not always saved to flash
 * Fix numerous stability issues
 * Firmware now checks for compatible NAP version
 * Added monitoring of UART throughput and observation latency
 * New compacted observation format for increased throughput and flexible over
   the wire frame size
 * Refactored UART code to allow peripherals (e.g. Bluetooth or 3G modules) to
   claim a UART for exclusive use
 * Remove the old Float Kalman Filter with position states that was running in
   the background, now we exclusively run the new KF which only uses ambiguity
   states and constraint equations
 * Add FLL aiding to the carrier tracking loop PLL during the initial pull-in
   period
 * Add ability to periodically transmit a message indicating the surveyed
   location of a base station
 * Add support for external modules to the build system
 * Add mechanism to register NMEA dispatchers (used by the external iAP
   Bluetooth module to interface to Apple devices)
 * If we have received a message with the surveyed location of the base station
   then output "pseudo-absolute" NMEA messages

#### libswiftnav

 * Generate a fixed solution even if only a partial set of PRNs have had their
   integer ambiguity resolved
 * Add FLL-aided-PLL loop filter
 * Fixed bug where the reference satellite was not found
 * Numerous bug fixes and stability improvements

#### Console

 * Check for updated Piksi firmware, NAP firmware and console version
   on start-up and prompt the user to flash updated versions to the device
 * Allow the user to select a serial port using a GUI dialog if one is not
   specified on the command line
 * Binary builds of the console now available for Window and Mac OS X
 * Fixed a bug with the RINEX header format output
 * Numerous fixes to support a wider range of Python/Qt/Wx environments
 * Numerous cosmetic enhancements
 * Format of CSV log files improved for greater precision and more readable
   time format

v0.8 <a name="v0.8"></a>
----

 * First public beta release, as shipped with Kickstarter rewards and pre-order
   hardware
 * Basic RTK functionality

