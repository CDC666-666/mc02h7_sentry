const http = require('http');
const fs = require('fs');
const path = require('path');

const root = __dirname;
const contentTypes = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.md': 'text/plain; charset=utf-8'
};

http.createServer((request, response) => {
  const requested = request.url === '/' ? '/index.html' : decodeURIComponent(request.url.split('?')[0]);
  const file = path.resolve(root, '.' + requested);
  if (!file.startsWith(root + path.sep)) {
    response.writeHead(403);
    response.end('Forbidden');
    return;
  }
  fs.readFile(file, (error, content) => {
    if (error) {
      response.writeHead(error.code === 'ENOENT' ? 404 : 500);
      response.end(error.code === 'ENOENT' ? 'Not found' : 'Read error');
      return;
    }
    response.writeHead(200, {
      'Content-Type': contentTypes[path.extname(file)] || 'application/octet-stream',
      'Cache-Control': 'no-store'
    });
    response.end(content);
  });
}).listen(8001, '127.0.0.1', () => {
  console.log('DM-IMU-L1 debugger: http://127.0.0.1:8001');
});
