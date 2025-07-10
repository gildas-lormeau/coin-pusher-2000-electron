import { join, extname } from "node:path";
import { fileURLToPath } from "node:url";
import { stat, readFile } from "node:fs/promises";
import { app, BrowserWindow, protocol } from "electron";

const __dirname = fileURLToPath(new URL(".", import.meta.url));

app.whenReady().then(() => {
    protocol.handle("file", async (request) => {
        const url = new URL(request.url);
        let filePath = fileURLToPath(url);
        const fileExists = await exists(filePath);
        if (!fileExists && url.pathname.startsWith(__dirname)) {
            filePath = join(__dirname, "game", "public", url.pathname.substring(__dirname.length));
            const fileExists = await exists(filePath);
            if (!fileExists) {
                return new Response("File not found", { status: 404 });
            }
        }
        try {
            const ext = extname(filePath).toLowerCase();
            let mimeType = "application/octet-stream";
            const mimeTypes = {
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
            if (mimeTypes[ext]) {
                mimeType = mimeTypes[ext];
            }
            return new Response(await readFile(filePath), { headers: { "Content-Type": mimeType } });
        } catch (error) {
            return new Response("File not found", { status: 404 });
        }
    });

    createWindow();
});

async function exists(filePath) {
    try {
        await stat(filePath);
        return true;
    } catch {
        return false;
    }
}

const createWindow = () => {
    const mainWindow = new BrowserWindow({
        fullscreen: true,
        webPreferences: {
            nodeIntegration: true,
            contextIsolation: false,
            enableRemoteModule: true,
            webSecurity: false
        }
    });
    mainWindow.loadFile("./index.html");
};

app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
});

app.on("window-all-closed", () => {
    if (process.platform !== "darwin") app.quit();
});
