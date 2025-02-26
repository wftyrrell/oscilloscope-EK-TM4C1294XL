# Oscilloscope Project for EK-TM4C1294XL

This project is an oscilloscope application for the EK-TM4C1294XL LaunchPad with the BOOSTXL-EDUMKII BoosterPack. It includes various functionalities such as sampling, button handling, and LCD display control.

## Project Structure

. ├── .ccsproject ├── .cproject ├── .gitignore ├── .launches/ │ ├── ece3849_lab0_starter.launch │ ├── ece3849b22_lab0_wftyrrell.launch │ ├── ece3849b22_lab1_wftyrrell.launch ├── .project ├── .settings/ │ ├── org.eclipse.cdt.codan.core.prefs │ ├── org.eclipse.cdt.debug.core.prefs │ ├── org.eclipse.core.resources.prefs ├── .vscode/ ├── buttons.c ├── buttons.h ├── Crystalfontz128x128_ST7735.c ├── Crystalfontz128x128_ST7735.h ├── Debug/ ├── HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.c ├── HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.h ├── main.c ├── README.md ├── sampling.c ├── sampling.h ├── sysctl_pll.c ├── sysctl_pll.h ├── targetConfigs/ │ ├── Tiva TM4C1294NCPDT.ccxml │ ├── readme.txt ├── tm4c1294ncpdt_startup_ccs.c ├── tm4c1294ncpdt.cmd

## Files Description

- **buttons.c / buttons.h**: Handles button inputs.
- **Crystalfontz128x128_ST7735.c / Crystalfontz128x128_ST7735.h**: Manages the Crystalfontz128x128 LCD display.
- **HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.c / HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.h**: Hardware abstraction layer for the LCD display.
- **main.c**: Main application code.
- **sampling.c / sampling.h**: Handles data sampling.
- **sysctl_pll.c / sysctl_pll.h**: System control and PLL configuration.
- **tm4c1294ncpdt_startup_ccs.c**: Startup code for the TM4C1294NCPDT microcontroller.
- **tm4c1294ncpdt.cmd**: Linker command file.

## Getting Started

1. **Clone the repository**:
    ```sh
    git clone <repository-url>
    ```

2. **Open the project in Code Composer Studio (CCS)**:
    - Import the project into CCS.
    - Ensure the target configuration is set to `Tiva TM4C1294NCPDT`.

3. **Build the project**:
    - Use the build options in CCS to compile the project.

4. **Load and run the project**:
    - Connect the EK-TM4C1294XL LaunchPad to your computer.
    - Load the compiled binary onto the LaunchPad using CCS.
    - Run the project and observe the oscilloscope functionality on the LCD display.

## License

This project is licensed under the BSD License. See the LICENSE file for details.

## Authors

- Gene Bogdanov

## Acknowledgments

- Texas Instruments for providing the hardware and software tools.
