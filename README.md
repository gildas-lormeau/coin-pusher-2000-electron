Warning: This is a work in progress!

This repository contains the code required to run [Coin Pusher 2000](https://github.com/gildas-lormeau/coin-pusher-2000) in Electron. This project mainly contains the Rapier.rs binding needed to run the game.

# Build

- Install Git if not already installed, cf. https://git-scm.com/
- Intall Node.js if not already installed, cf. https://nodejs.org/en/download
- Install Rust if not already installed, cf. https://www.rust-lang.org/tools/install
- Install Python (required for native module compilation):
  - **macOS**: `brew install python3` or download from https://www.python.org/
  - **Windows**: Download from https://www.python.org/ or `winget install Python.Python.3`
  - **Linux**: `sudo apt install python3 python3-dev` (Ubuntu/Debian) or equivalent
- Install build tools:
  - **macOS**: `xcode-select --install` (Command Line Tools)
  - **Windows**: Install Visual Studio Build Tools or Visual Studio Community
  - **Linux**: `sudo apt install build-essential` (Ubuntu/Debian) or equivalent

- Run the following commands to clone the project, install its dependencies, and build it
```sh
git clone --recursive https://github.com/gildas-lormeau/coin-pusher-2000-electron.git
npm install
npm run build
```
# Run

Launch the following command to run the game. 
```sh
npm start
```

# Package

Launch the following command to run the game. 
```sh
npm run package
```

# Troubleshooting

If you encounter build errors:
- Make sure Rust is properly installed: `rustc --version`
- Ensure Python is available: `python3 --version`
- On macOS, make sure Xcode Command Line Tools are installed
- On Windows, ensure Visual Studio Build Tools are installed
- Try clearing node modules: `rm -rf node_modules package-lock.json && npm install`