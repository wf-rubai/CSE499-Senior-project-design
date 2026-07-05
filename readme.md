# Autonomous Multi-Device Rescue Robot and Localisation

Every year between 26,000 and 31,000 square kilometers of Bangladesh’s land area is flooded during monsoon. Flood disasters create dangerous and unpredictable environments that make rescue operations difficult for human responders. It is because of strong water currents, poor visibility, and damaged communication systems. This project presents the design and implementation of a low-cost semi-autonomous flood rescue robotic system, capable of assisting rescue teams during emergency situations. The proposed system consists of two main units: an autonomous search robot and a monitoring station (HQ). The robot is designed using a catamaran-inspired floating structure to provide improved stability, buoyancy, and load distribution while navigating through floodwaters. The system integrates a Raspberry Pi Pico microcontroller, ESP32-CAM modules for image-based human detection, a UBlox NEO M8N GPS module for real-time location tracking, and an SX1278 LoRa module for long-range wireless communication. During operation, the robot continuously captures environmental and positional data while searching for stranded survivors and transmits this information to the HQ through LoRa communication. CSV packet transmission was selected for this project, due to  requiring less memory and bandwidth compared to JSON, making it more suitable for low-power long-range communication systems. At the monitoring station, the received CSV data is converted into JSON format and processed by a custom-built web interface that displays the real-time locations, movement trajectories, transmitting boat IDs, detected survivor information, and estimated displacement regions on an interactive map. To improve the stability and reliability of the human detection system, a moving average smoothing function was implemented, which significantly reduced fluctuations in detection confidence values under different environmental conditions. Experimental results demonstrated that the smoothing process improved detection consistency, particularly between distances of 2 to 7 feet, while maintaining reliable communication over long distances using LoRa technology. Overall, the project demonstrates the feasibility of an energy-efficient, scalable, and cost-effective flood rescue robotic system that can assist emergency responders and potentially support future swarm-based rescue operations in challenging disaster environments.

## Features

- Feature 1
- Feature 2
- Feature 3

## Requirements

- Requirement 1
- Requirement 2

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/your-repository.git
   ```
2. Navigate to the project directory:
   ```bash
   cd your-repository
   ```
3. Install the required dependencies.

## Usage

Run the project using the appropriate command:

```bash
# Example
python main.py
```

or

```bash
npm start
```

## Project Structure

```
project/
├── src/
├── assets/
├── docs/
├── README.md
└── LICENSE
```

## Contributing

Contributions are welcome! Feel free to fork the repository, create a new branch, and submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more information.

## Author

Your Name

GitHub: https://github.com/your-username