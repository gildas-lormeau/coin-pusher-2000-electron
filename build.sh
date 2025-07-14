npm run build
rm -rf build
npx vite build
cp package.json build
cp index.js build
mkdir -p build/rapier3d-f64-electron
cp rapier3d-f64-electron/index.js build/rapier3d-f64-electron/
cp rapier3d-f64-electron/index.node build/rapier3d-f64-electron/
cp rapier3d-f64-electron/package.json build/rapier3d-f64-electron/
mkdir -p build/rapier3d-f64-electron/target/release
cp rapier3d-f64-electron/target/release/librapier3d* build/rapier3d-f64-electron/target/release/
cp game/public/assets/* build/assets
rm build/assets/*.blend
rm build/assets/*.xcf
rm build/assets/*.txt
rm build/assets/sunset.png
rm build/assets/reel-icons.png
find build/assets -type f -name "*.exr" ! -name "sunset.exr" -delete
mkdir -p build/views
cp game/public/views/* build/views
cd build
npm install
CSC_IDENTITY_AUTO_DISCOVERY=false npx electron-builder --mac --arm64 --x64
cd ..
rm -rf dist
mkdir -p dist
cp -R build/dist/* dist
rm -rf build