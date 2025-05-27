import React, { useEffect, useState } from 'react';
import './App.css';

const ROWS = 6;
const COLUMNS = 8;

function parsePath(pathString) {
  return pathString
    .split(' > ')
    .map(pair => {
      const match = pair.match(/\((\d+),\s*(\d+)\)/);
      return match ? { x: parseInt(match[1]), y: parseInt(match[2]) } : null;
    })
    .filter(Boolean);
}

function parseObstacles(obstacleString) {
  const matches = [...obstacleString.matchAll(/\((\d+),\s*(\d+)\)/g)];
  return matches.map(match => ({
    x: parseInt(match[1]),
    y: parseInt(match[2]),
  }));
}


function App() {
  const [path, setPath] = useState([]);
  const [obstacles, setObstacles] = useState([]);

  const url = "https://api.eu-w1.tago.io/data"
  const headers = {
    "Content-Type": "application/json",
    "Device-Token": "2d40a1f9-7dda-4e91-8240-c1bd40693a41",
  }

  const fetchData = async () => {
    try {
      const response = await fetch(url, {
        method: 'GET',
        headers: headers,
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const result = await response.json();

      const sorted = result.result.sort((a, b) => new Date(b.time) - new Date(a.time));

      const pathItem = sorted.find((item) => item.variable === 'path');
      const obstaclesItem = sorted.find((item) => item.variable === 'obstacles');

      if (pathItem && pathItem.value && JSON.stringify(pathItem.value) !== JSON.stringify(path)) {
        console.log("New path detected, updating state.", pathItem.value);
        setPath(parsePath(pathItem.value));
      }

      if (obstaclesItem && obstaclesItem.value && JSON.stringify(obstaclesItem.value) !== JSON.stringify(obstacles)) {
        console.log("New obstacles detected, updating state.", obstaclesItem.value);
        setObstacles(parseObstacles(obstaclesItem.value));
      }

    } catch (error) {
      console.error("Error fetching data:", error);
    }
  }

  useEffect(() => {
    fetchData();
    const interval = setInterval(fetchData, 1000);
    return () => clearInterval(interval); 
  }
  , []);

  // const obstacles = parseObstacles(obstacleString);

  // const path = parsePath(pathString);
  const end = path[0] || { x: 0, y: 0 }; 
  const start = path[path.length - 1] || { x: ROWS - 1, y: COLUMNS - 1 };

  const getCell = (row, col) => {
    if (row === start.x && col === start.y) return "start";
    if (row === end.x && col === end.y) return "end";
    if (path.some(p => p.x === row && p.y === col)) return "path";
    if (obstacles.some(o => o.x === row && o.y === col)) return "obstacle";
    return "empty";
  };

  return (
    <div className="app-container">
      <h1>Most Optimal Path</h1>
      <div className="grid">
        {[...Array(ROWS)].map((_, row) =>
          [...Array(COLUMNS)].map((_, col) => (
            <div
              key={`${row}-${col}`}
              className={`cell ${getCell(row, col)}`}
            />
          ))
        )}
      </div>

      <div className="info">
        <p><strong>Path:</strong> {path.map(p => `(${p.x}, ${p.y})`).join(' < ')}</p>
        <p><strong>Obstacles:</strong> {obstacles.map(o => `(${o.x}, ${o.y})`).join(', ')}</p>
      </div>

      <div className="legend">
        <div><span className="legend-box start"></span> Start</div>
        <div><span className="legend-box end"></span> End</div>
        <div><span className="legend-box path"></span> Path</div>
        <div><span className="legend-box obstacle"></span> Obstacle</div>
        <div><span className="legend-box empty"></span> Empty</div>
      </div>
    </div>
  );
}

export default App;
