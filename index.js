import { join, extname } from "node:path";
import { fileURLToPath } from "node:url";
import { stat, readFile } from "node:fs/promises";
import { app, BrowserWindow, protocol } from "electron";

const MIME_TYPES = {
    ".html": "text/html",
    ".js": "application/javascript",
    ".css": "text/css",
    ".glb": "model/gltf-binary",
    ".exr": "image/x-exr",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".woff2": "font/woff2",
    ".woff": "font/woff",
    ".json": "application/json"
};
const DEFAULT_MIME_TYPE = "application/octet-stream";
const CONTENT_TYPE_HEADER = "Content-Type";
const FILE_NOT_FOUND_RESPONSE = new Response("File not found", { status: 404 });
const APP_DIRNAME = fileURLToPath(new URL(".", import.meta.url));
const PUBLIC_DIRNAME = join(APP_DIRNAME, "game", "public");
const APP_PATHNAME = "./index.html";
main();

async function main() {
    app.on("activate", onActivate);
    app.on("window-all-closed", onClose);
    await app.whenReady();
    protocol.handle("file", onFileProtocol);
    await createWindow();
}

async function onActivate() {
    if (BrowserWindow.getAllWindows().length === 0) {
        await createWindow();
    }
}

function onClose() {
    app.quit();
}

async function onFileProtocol(request) {
    const url = new URL(request.url);
    let filePath = fileURLToPath(url);
    if (!await exists(fileURLToPath(url))) {
        if (url.pathname.startsWith(APP_DIRNAME)) {
            filePath = join(PUBLIC_DIRNAME, url.pathname.substring(APP_DIRNAME.length));
            if (!await exists(filePath)) {
                return FILE_NOT_FOUND_RESPONSE;
            }
        } else {
            return FILE_NOT_FOUND_RESPONSE;
        }
    }
    const fileExtension = extname(filePath).toLowerCase();
    const mimeType = MIME_TYPES[fileExtension] ? MIME_TYPES[fileExtension] : DEFAULT_MIME_TYPE;
    return new Response(await readFile(filePath), { headers: { [CONTENT_TYPE_HEADER]: mimeType } });
}

async function createWindow() {
    const mainWindow = new BrowserWindow({
        fullscreen: true,
        webPreferences: {
            nodeIntegration: true,
            contextIsolation: false,
            enableRemoteModule: true,
            webSecurity: false
        }
    });
    await mainWindow.loadFile(APP_PATHNAME);
}

async function exists(filePath) {
    try {
        await stat(filePath);
        return true;
    } catch {
        return false;
    }
}