npm run build
rm -rf build
npx vite build
cp package.json build
cp index.js build
cp -R rapier3d-electron build/rapier3d-electron
cp -R rapier3d-f64-electron build/rapier3d-f64-electron
cp game/public/assets/* build/assets
mkdir -p build/views
cp game/public/views/* build/views
cd build
npm install
CSC_IDENTITY_AUTO_DISCOVERY=false npx electron-builder
cd ..
rm -rf dist
mkdir -p dist
cp -R build/dist/* dist
rm -rf build