// E-Rubab websocket server v0.7.1
// by Casijn Broerse, 2025
// License MIT

import express from 'express';
import { createServer } from 'node:http';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { Server } from 'socket.io';
//import sqlite3 from 'sqlite3';
//import { open } from 'sqlite';

// open the database file to resend any missed messages after reconnect
//const db = await open({
//  filename: 'chat.db',
//  driver: sqlite3.Database
//});

// create our 'messages' table
//await db.exec(`
//  CREATE TABLE IF NOT EXISTS messages (
//      id INTEGER PRIMARY KEY AUTOINCREMENT,
//      client_offset TEXT UNIQUE,
//      content TEXT
//  );
//`);

const app = express();
const server = createServer(app);
const io = new Server(server, {
  connectionStateRecovery: {}  // will retry to connect if connection dropped
});

const __dirname = dirname(fileURLToPath(import.meta.url));

app.get('/', (req, res) => {
  // for browsers:
  res.sendFile(join(__dirname, 'index.html'));
});

// When the server receives a post request on /sendData
app.post('/sendData', function (req, res) {

    //send data to sockets.
    io.of("/rubab").sockets.emit('event', { message: "Hello from server!" })
    
    res.send({});
});

io.on('connection', async (socket) => {
  console.log('a user connected');
  
  socket.on('disconnect', () => {
    console.log('a user disconnected');
  });
  
  socket.on('connect error', () => {
    console.log('connection error');
  });
  
  // our ESP32 Arduino Rubab note event
  socket.on('rb_note', (msg) => {
    console.log('rb_note event message: str ' + msg.data[0] + ', frt ' + msg.data[1] + ' src:' + msg.data[2]); // show JSON fields
    io.emit('note', msg); // forwards renamed note msg to all connected
  });
  
  // volgende blokje vervangen in index-rb.js:
  socket.on('rb_notestring', (msg) => {
    console.log('rb_notestring event message: ' + msg);
    io.emit('note', msg); // forwards renamed note msg to all connected
    io.emit('menu', msg); // forwards renamed note msg as menu choice to all connected
  });
  
  // originale example chat in browser
  socket.on('chat message', async (msg, clientOffset, callback) => {
    //let result;
    //try {
    //  result = await db.run('INSERT INTO messages (content, client_offset) VALUES (?, ?)', msg, clientOffset);
    //} catch (e) {
    //  if (e.errno === 19 /* SQLITE_CONSTRAINT */ ) {
    //    // the message was already inserted, so we notify the client
    //    callback();
    //  } else {
    //    // nothing to do, just let the client retry
    //  }
    //  return;
    //}
    console.log('chat event message: ' + msg);
    io.emit('chat message', msg, result.lastID);
    // acknowledge the event
    callback();
  });

//  if (!socket.recovered) {
//    try {
//      await db.each('SELECT id, content FROM messages WHERE id > ?',
//        [socket.handshake.auth.serverOffset || 0],
//        (_err, row) => {
//          socket.emit('chat message', row.content, row.id);
//        }
//      )
//    } catch (e) {
//      // something went wrong
//    }
//  }
});
  
server.listen(3000, () => {
  console.log('server running at http://localhost:3000');
});