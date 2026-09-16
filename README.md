# Pixel Platformer

A browser-based platformer game built with HTML5 Canvas and vanilla JavaScript. No external assets required - everything is procedurally generated!

## Features

- **Procedural Graphics**: All game art is drawn programmatically using canvas primitives
- **Synthesized Sound Effects**: Web Audio API for jump, coin, damage, and win sounds
- **Particle Effects**: Dust, sparkles, and death animations
- **Smooth Physics**: Fixed-timestep game loop with proper collision detection
- **Responsive Controls**: Keyboard (WASD/Arrows) and touch support for mobile
- **Multiple Enemy Types**: Slimes, chasers, and flying bats
- **Collectible System**: Coins scattered throughout the level
- **Health System**: Take damage from enemies, survive to win!

## How to Play

1. Open `index.html` in any modern web browser
2. Use **WASD** or **Arrow Keys** to move
3. Press **Space** or **Up Arrow** to jump
4. Collect all coins to win
5. Avoid touching enemies!

## Project Structure

```
project/
├── src/
│   ├── index.html      # Main HTML file
│   └── game.js         # Complete game logic
├── package.json        # Project configuration
├── .editorconfig       # Editor settings
└── .gitignore          # Git exclusions
```

## Technologies Used

- **HTML5 Canvas**: For all rendering
- **Web Audio API**: For synthesized sound effects
- **Vanilla JavaScript**: No frameworks or libraries required

## Running Locally

```bash
# Install dependencies
npm install

# Start development server
npm run dev
```

Then open http://localhost:3000 in your browser.

## Game Feel Enhancements

- Screen shake on impact
- Hit flash effects
- Particle bursts for jumps and deaths
- Squash-and-stretch on landing
- Floating score text
- Smooth animations

## Future Enhancements

- Multiple levels
- Boss battles
- Power-ups
- More enemy types
- Combo system
- Leaderboards

## License

MIT License - Feel free to use, modify, and distribute!

---

Built with ❤️ by the Coder agent