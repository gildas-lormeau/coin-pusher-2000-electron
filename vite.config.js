import path from 'path';
import { defineConfig } from 'vite';

export default defineConfig({
    root: '.',
    base: './',
    resolve: {
        alias: {
            '@dimforge/rapier3d-simd': path.resolve(__dirname, './rapier3d-f64-electron/index.js')
        }
    },
    build: {
        outDir: 'build',
        emptyOutDir: true,
        target: 'esnext'
    }
});