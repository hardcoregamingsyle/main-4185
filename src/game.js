// Pixel Platformer - Main Game File
(function() {
    'use strict';
    
    // ============================================
    // AUDIO SYSTEM (Web Audio API)
    // ============================================
    const AudioSystem = {
        ctx: null,
        initialized: false,
        
        init() {
            if (this.initialized) return;
            try {
                this.ctx = new (window.AudioContext || window.webkitAudioContext)();
                this.initialized = true;
            } catch (e) {
                console.warn('Audio not supported');
            }
        },
        
        resume() {
            if (this.ctx && this.ctx.state === 'suspended') {
                this.ctx.resume();
            }
        },
        
        playJump() {
            if (!this.initialized) return;
            const osc = this.ctx.createOscillator();
            const gain = this.ctx.createGain();
            osc.connect(gain);
            gain.connect(this.ctx.destination);
            
            osc.frequency.setValueAtTime(400, this.ctx.currentTime);
            osc.frequency.exponentialRampToValueAtTime(600, this.ctx.currentTime + 0.1);
            
            gain.gain.setValueAtTime(0.3, this.ctx.currentTime);
            gain.gain.exponentialRampToValueAtTime(0.01, this.ctx.currentTime + 0.1);
            
            osc.start();
            osc.stop(this.ctx.currentTime + 0.1);
        },
        
        playCoin() {
            if (!this.initialized) return;
            const osc = this.ctx.createOscillator();
            const gain = this.ctx.createGain();
            osc.connect(gain);
            gain.connect(this.ctx.destination);
            
            osc.type = 'sine';
            osc.frequency.setValueAtTime(1200, this.ctx.currentTime);
            osc.frequency.exponentialRampToValueAtTime(1800, this.ctx.currentTime + 0.1);
            
            gain.gain.setValueAtTime(0.2, this.ctx.currentTime);
            gain.gain.exponentialRampToValueAtTime(0.01, this.ctx.currentTime + 0.1);
            
            osc.start();
            osc.stop(this.ctx.currentTime + 0.1);
        },
        
        playDamage() {
            if (!this.initialized) return;
            const osc = this.ctx.createOscillator();
            const gain = this.ctx.createGain();
            osc.connect(gain);
            gain.connect(this.ctx.destination);
            
            osc.type = 'square';
            osc.frequency.setValueAtTime(150, this.ctx.currentTime);
            osc.frequency.linearRampToValueAtTime(50, this.ctx.currentTime + 0.2);
            
            gain.gain.setValueAtTime(0.3, this.ctx.currentTime);
            gain.gain.exponentialRampToValueAtTime(0.01, this.ctx.currentTime + 0.2);
            
            osc.start();
            osc.stop(this.ctx.currentTime + 0.2);
        },
        
        playWin() {
            if (!this.initialized) return;
            const now = this.ctx.currentTime;
            const notes = [523, 659, 784, 1047];
            
            notes.forEach((freq, i) => {
                const osc = this.ctx.createOscillator();
                const gain = this.ctx.createGain();
                osc.connect(gain);
                gain.connect(this.ctx.destination);
                
                osc.type = 'triangle';
                osc.frequency.value = freq;
                
                gain.gain.setValueAtTime(0.3, now + i * 0.1);
                gain.gain.exponentialRampToValueAtTime(0.01, now + i * 0.1 + 0.3);
                
                osc.start(now + i * 0.1);
                osc.stop(now + i * 0.1 + 0.3);
            });
        }
    };
    
    // ============================================
    // PARTICLE SYSTEM
    // ============================================
    class Particle {
        constructor(x, y, color, vx, vy, life) {
            this.x = x;
            this.y = y;
            this.color = color;
            this.vx = vx;
            this.vy = vy;
            this.life = life;
            this.maxLife = life;
            this.size = Math.random() * 4 + 2;
        }
        
        update(dt) {
            this.x += this.vx * dt;
            this.y += this.vy * dt;
            this.life -= dt;
            this.vy += 50 * dt; // gravity
        }
        
        draw(ctx) {
            const alpha = this.life / this.maxLife;
            ctx.globalAlpha = alpha;
            ctx.fillStyle = this.color;
            ctx.beginPath();
            ctx.arc(this.x, this.y, this.size, 0, Math.PI * 2);
            ctx.fill();
            ctx.globalAlpha = 1;
        }
        
        isDead() {
            return this.life <= 0;
        }
    }
    
    // ============================================
    // INPUT HANDLER
    // ============================================
    class InputHandler {
        constructor() {
            this.keys = {};
            this.touchX = null;
            this.touchY = null;
            this.jumpPressed = false;
            
            window.addEventListener('keydown', (e) => {
                this.keys[e.key.toLowerCase()] = true;
                if (e.code === 'Space' || e.code === 'ArrowUp') {
                    this.jumpPressed = true;
                }
            });
            
            window.addEventListener('keyup', (e) => {
                this.keys[e.key.toLowerCase()] = false;
                if (e.code === 'Space' || e.code === 'ArrowUp') {
                    this.jumpPressed = false;
                }
            });
            
            const canvas = document.getElementById('game-canvas');
            
            canvas.addEventListener('touchstart', (e) => {
                e.preventDefault();
                this.touchX = e.touches[0].clientX;
                this.touchY = e.touches[0].clientY;
                this.jumpPressed = true;
                AudioSystem.resume();
            });
            
            canvas.addEventListener('touchmove', (e) => {
                e.preventDefault();
                this.touchX = e.touches[0].clientX;
            });
            
            canvas.addEventListener('touchend', (e) => {
                e.preventDefault();
                this.jumpPressed = false;
            });
            
            canvas.addEventListener('click', () => {
                AudioSystem.resume();
            });
        }
        
        getLeft() {
            return this.keys['arrowleft'] || this.keys['a'];
        }
        
        getRight() {
            return this.keys['arrowright'] || this.keys['d'];
        }
        
        getUp() {
            return this.keys['arrowup'] || this.keys['w'];
        }
        
        getDown() {
            return this.keys['arrowdown'] || this.keys['s'];
        }
        
        getJump() {
            return this.jumpPressed;
        }
        
        getRestart() {
            return this.keys['r'] || this.keys['enter'];
        }
    }
    
    // ============================================
    // GAME ENTITIES
    // ============================================
    class Player {
        constructor(x, y) {
            this.x = x;
            this.y = y;
            this.width = 32;
            this.height = 48;
            this.vx = 0;
            this.vy = 0;
            this.speed = 200;
            this.jumpStrength = 450;
            this.gravity = 1200;
            this.friction = 0.85;
            this.onGround = false;
            this.facingRight = true;
            this.health = 100;
            this.maxHealth = 100;
            this.invulnerable = 0;
            this.animFrame = 0;
            this.animTimer = 0;
        }
        
        update(dt, input, tiles, enemies) {
            // Horizontal movement
            if (input.getLeft()) {
                this.vx -= this.speed * dt;
                this.facingRight = false;
            }
            if (input.getRight()) {
                this.vx += this.speed * dt;
                this.facingRight = true;
            }
            
            // Apply friction
            this.vx *= this.friction;
            
            // Clamp velocity
            this.vx = Math.max(-this.speed, Math.min(this.speed, this.vx));
            
            // Vertical movement (jump)
            if (input.getJump() && this.onGround) {
                this.vy = -this.jumpStrength;
                this.onGround = false;
                AudioSystem.playJump();
                this.spawnParticles(4);
            }
            
            // Gravity
            this.vy += this.gravity * dt;
            
            // Update position
            this.x += this.vx * dt;
            this.y += this.vy * dt;
            
            // Collision detection
            this.onGround = false;
            this.checkTileCollision(tiles);
            this.checkEnemyCollision(enemies);
            
            // Screen bounds
            this.x = Math.max(0, Math.min(800 - this.width, this.x));
            this.y = Math.max(0, Math.min(600 - this.height, this.y));
            
            // Invulnerability timer
            if (this.invulnerable > 0) {
                this.invulnerable -= dt;
            }
            
            // Animation timer
            this.animTimer += dt;
            if (this.animTimer > 0.15) {
                this.animFrame = (this.animFrame + 1) % 4;
                this.animTimer = 0;
            }
        }
        
        checkTileCollision(tiles) {
            // Simple AABB collision with tiles
            for (const tile of tiles) {
                if (tile.type !== 'solid') continue;
                
                if (this.x < tile.x + tile.width &&
                    this.x + this.width > tile.x &&
                    this.y < tile.y + tile.height &&
                    this.y + this.height > tile.y) {
                    
                    // Determine collision side
                    const dx = (this.x + this.width / 2) - (tile.x + tile.width / 2);
                    const dy = (this.y + this.height / 2) - (tile.y + tile.height / 2);
                    const width = (this.width + tile.width) / 2;
                    const height = (this.height + tile.height) / 2;
                    const crossWidth = width * dy;
                    const crossHeight = height * dx;
                    
                    if (Math.abs(dx) <= width && Math.abs(dy) <= height) {
                        if (crossWidth > crossHeight) {
                            if (crossWidth > -crossHeight) {
                                // Bottom collision
                                this.y = tile.y + tile.height;
                                this.vy = 0;
                            } else {
                                // Left collision
                                this.x = tile.x - this.width;
                                this.vx = 0;
                            }
                        } else {
                            if (crossWidth > -crossHeight) {
                                // Right collision
                                this.x = tile.x + tile.width;
                                this.vx = 0;
                            } else {
                                // Top collision (ground)
                                this.y = tile.y - this.height;
                                this.vy = 0;
                                this.onGround = true;
                            }
                        }
                    }
                }
            }
        }
        
        checkEnemyCollision(enemies) {
            for (const enemy of enemies) {
                if (enemy.dead) continue;
                
                if (this.x < enemy.x + enemy.width &&
                    this.x + this.width > enemy.x &&
                    this.y < enemy.y + enemy.height &&
                    this.y + this.height > enemy.y) {
                    
                    if (this.invulnerable <= 0) {
                        this.takeDamage(enemy.damage);
                        
                        // Bounce back
                        const bounceDir = this.x < enemy.x ? -1 : 1;
                        this.vx = bounceDir * 200;
                        this.vy = -200;
                    }
                }
            }
        }
        
        takeDamage(amount) {
            this.health -= amount;
            this.invulnerable = 1.5;
            AudioSystem.playDamage();
            this.spawnParticles(8, '#ff4444');
            updateHealthUI();
            
            if (this.health <= 0) {
                gameOver();
            }
        }
        
        spawnParticles(count, color = '#ffffff') {
            for (let i = 0; i < count; i++) {
                const angle = Math.random() * Math.PI * 2;
                const speed = Math.random() * 100 + 50;
                particles.push(new Particle(
                    this.x + this.width / 2,
                    this.y + this.height / 2,
                    color,
                    Math.cos(angle) * speed,
                    Math.sin(angle) * speed,
                    0.5 + Math.random() * 0.5
                ));
            }
        }
        
        draw(ctx) {
            if (this.invulnerable > 0 && Math.floor(Date.now() / 50) % 2 === 0) {
                return; // Flicker when invulnerable
            }
            
            const cx = this.x + this.width / 2;
            const cy = this.y + this.height / 2;
            
            // Body
            ctx.fillStyle = '#4CAF50';
            ctx.fillRect(this.x + 4, this.y + 20, this.width - 8, this.height - 24);
            
            // Head
            ctx.fillStyle = '#FFCC80';
            ctx.fillRect(this.x + 8, this.y, this.width - 16, 20);
            
            // Eyes
            ctx.fillStyle = '#333';
            const eyeOffset = this.facingRight ? 4 : -4;
            ctx.fillRect(this.x + 14 + eyeOffset, this.y + 5, 4, 6);
            ctx.fillRect(this.x + 22 + eyeOffset, this.y + 5, 4, 6);
            
            // Legs animation
            const legOffset = Math.sin(this.animFrame * Math.PI / 2) * 4;
            ctx.fillStyle = '#2E7D32';
            ctx.fillRect(this.x + 6, this.y + this.height - 10, 8, 10 + legOffset);
            ctx.fillRect(this.x + this.width - 14, this.y + this.height - 10, 8, 10 - legOffset);
        }
    }
    
    class Enemy {
        constructor(x, y, type = 'patrol') {
            this.x = x;
            this.y = y;
            this.width = 32;
            this.height = 32;
            this.type = type;
            this.health = 1;
            this.dead = false;
            this.damage = 20;
            
            if (type === 'slime') {
                this.color = '#9C27B0';
                this.speed = 80;
                this.hp = 2;
            } else if (type === 'bat') {
                this.color = '#E91E63';
                this.speed = 100;
                this.hp = 1;
                this.flying = true;
            } else {
                this.color = '#F44336';
                this.speed = 60;
                this.hp = 3;
            }
            
            this.direction = 1;
            this.patrolStart = x;
            this.patrolDistance = 100;
            this.jumpTimer = 0;
        }
        
        update(dt, tiles) {
            if (this.dead) return;
            
            if (this.type === 'patrol') {
                // Patrol behavior
                this.x += this.speed * this.direction * dt;
                
                // Turn around at patrol limits
                if (this.x - this.patrolStart > this.patrolDistance || 
                    this.x - this.patrolStart < -this.patrolDistance) {
                    this.direction *= -1;
                }
                
                // Random jump
                this.jumpTimer -= dt;
                if (this.jumpTimer <= 0 && Math.random() < 0.02) {
                    this.vy = -250;
                    this.jumpTimer = 2 + Math.random() * 3;
                }
            } else if (this.type === 'chase') {
                // Chase player (basic AI)
                const dx = player.x - this.x;
                if (Math.abs(dx) < 300) {
                    this.direction = Math.sign(dx);
                    this.x += this.speed * this.direction * dt;
                }
            }
            
            // Apply gravity
            if (this.flying) {
                this.vy += this.gravity * dt;
                this.y += this.vy * dt;
            } else {
                this.vy += this.gravity * dt;
                this.y += this.vy * dt;
                
                // Ground collision
                for (const tile of tiles) {
                    if (tile.type !== 'solid') continue;
                    
                    if (this.x < tile.x + tile.width &&
                        this.x + this.width > tile.x &&
                        this.y < tile.y + tile.height &&
                        this.y + this.height > tile.y) {
                        
                        if (this.vy > 0 && this.y + this.height - this.vy * dt <= tile.y) {
                            this.y = tile.y - this.height;
                            this.vy = 0;
                            this.jumpTimer = 0;
                        }
                    }
                }
            }
            
            // Screen bounds
            this.x = Math.max(0, Math.min(800 - this.width, this.x));
            this.y = Math.max(0, Math.min(600 - this.height, this.y));
        }
        
        takeDamage(amount) {
            this.health -= amount;
            this.hp -= amount;
            
            if (this.hp <= 0) {
                this.dead = true;
                score += 100;
                AudioSystem.playCoin();
                this.spawnDeathParticles();
            } else {
                AudioSystem.playDamage();
                this.spawnDamageParticles();
            }
            
            updateScoreUI();
        }
        
        spawnDeathParticles() {
            for (let i = 0; i < 10; i++) {
                const angle = Math.random() * Math.PI * 2;
                const speed = Math.random() * 150 + 50;
                particles.push(new Particle(
                    this.x + this.width / 2,
                    this.y + this.height / 2,
                    this.color,
                    Math.cos(angle) * speed,
                    Math.sin(angle) * speed,
                    0.8 + Math.random() * 0.4
                ));
            }
        }
        
        spawnDamageParticles() {
            for (let i = 0; i < 5; i++) {
                const angle = Math.random() * Math.PI * 2;
                const speed = Math.random() * 80 + 30;
                particles.push(new Particle(
                    this.x + this.width / 2,
                    this.y + this.height / 2,
                    '#ffffff',
                    Math.cos(angle) * speed,
                    Math.sin(angle) * speed,
                    0.3 + Math.random() * 0.2
                ));
            }
        }
        
        draw(ctx) {
            if (this.dead) return;
            
            // Body
            ctx.fillStyle = this.color;
            ctx.fillRect(this.x, this.y, this.width, this.height);
            
            // Eyes
            ctx.fillStyle = '#fff';
            const eyeOffset = this.direction > 0 ? 4 : -4;
            ctx.fillRect(this.x + 8 + eyeOffset, this.y + 8, 8, 8);
            ctx.fillRect(this.x + 16 + eyeOffset, this.y + 8, 8, 8);
            
            // Pupils
            ctx.fillStyle = '#000';
            ctx.fillRect(this.x + 10 + eyeOffset, this.y + 10, 4, 4);
            ctx.fillRect(this.x + 18 + eyeOffset, this.y + 10, 4, 4);
            
            // Health bar above head
            if (this.hp < 3) {
                ctx.fillStyle = '#333';
                ctx.fillRect(this.x, this.y - 8, this.width, 6);
                ctx.fillStyle = '#ff4444';
                ctx.fillRect(this.x, this.y - 8, this.width * (this.hp / 3), 6);
            }
        }
    }
    
    class Coin {
        constructor(x, y) {
            this.x = x;
            this.y = y;
            this.width = 20;
            this.height = 20;
            this.collected = false;
            this.bobOffset = Math.random() * Math.PI * 2;
        }
        
        update(dt) {
            this.bobOffset += dt * 5;
        }
        
        draw(ctx) {
            if (this.collected) return;
            
            const bobY = this.y + Math.sin(this.bobOffset) * 5;
            
            // Coin glow
            ctx.shadowColor = '#FFD700';
            ctx.shadowBlur = 15;
            
            // Coin body
            ctx.fillStyle = '#FFD700';
            ctx.beginPath();
            ctx.arc(this.x + this.width / 2, bobY + this.height / 2, 10, 0, Math.PI * 2);
            ctx.fill();
            
            ctx.shadowBlur = 0;
            
            // Inner detail
            ctx.fillStyle = '#FFA500';
            ctx.beginPath();
            ctx.arc(this.x + this.width / 2, bobY + this.height / 2, 6, 0, Math.PI * 2);
            ctx.fill();
            
            // Shine
            ctx.fillStyle = '#FFF';
            ctx.beginPath();
            ctx.arc(this.x + this.width / 2 - 3, bobY + this.height / 2 - 3, 2, 0, Math.PI * 2);
            ctx.fill();
        }
    }
    
    // ============================================
    // LEVEL DATA
    // ============================================
    const levelData = {
        tiles: [
            // Floor
            {x: 0, y: 550, width: 800, height: 50, type: 'solid'},
            
            // Platforms
            {x: 100, y: 450, width: 150, height: 20, type: 'solid'},
            {x: 300, y: 400, width: 150, height: 20, type: 'solid'},
            {x: 500, y: 350, width: 150, height: 20, type: 'solid'},
            {x: 150, y: 280, width: 100, height: 20, type: 'solid'},
            {x: 400, y: 250, width: 120, height: 20, type: 'solid'},
            {x: 600, y: 200, width: 150, height: 20, type: 'solid'},
            {x: 250, y: 150, width: 100, height: 20, type: 'solid'},
            
            // Walls
            {x: -20, y: 0, width: 20, height: 600, type: 'solid'},
            {x: 800, y: 0, width: 20, height: 600, type: 'solid'},
            
            // Blocks
            {x: 200, y: 350, width: 40, height: 40, type: 'solid'},
            {x: 350, y: 300, width: 40, height: 40, type: 'solid'},
            {x: 550, y: 250, width: 40, height: 40, type: 'solid'},
        ],
        
        enemies: [
            {x: 200, y: 520, type: 'slime'},
            {x: 400, y: 370, type: 'chase'},
            {x: 650, y: 170, type: 'bat'},
            {x: 50, y: 520, type: 'slime'},
        ],
        
        coins: [
            {x: 175, y: 420},
            {x: 375, y: 370},
            {x: 575, y: 320},
            {x: 200, y: 250},
            {x: 460, y: 220},
            {x: 675, y: 170},
            {x: 300, y: 120},
            {x: 100, y: 320},
            {x: 600, y: 320},
            {x: 400, y: 520},
        ]
    };
    
    // ============================================
    // GLOBAL VARIABLES
    // ============================================
    let canvas, ctx;
    let player;
    let tiles = [];
    let enemies = [];
    let coins = [];
    let particles = [];
    let score = 0;
    let coinCount = 0;
    let lastTime = 0;
    let gameRunning = false;
    let shakeAmount = 0;
    
    // ============================================
    // UI ELEMENTS
    // ============================================
    function initUI() {
        canvas = document.getElementById('game-canvas');
        ctx = canvas.getContext('2d');
        
        document.getElementById('start-btn').addEventListener('click', startGame);
        document.getElementById('restart-btn').addEventListener('click', restartGame);
        document.getElementById('play-again-btn').addEventListener('click', restartGame);
    }
    
    function showScreen(screenId) {
        document.querySelectorAll('.overlay').forEach(el => el.classList.add('hidden'));
        if (screenId) {
            document.getElementById(screenId).classList.remove('hidden');
        }
    }
    
    function updateScoreUI() {
        document.getElementById('score').textContent = score;
    }
    
    function updateCoinsUI() {
        document.getElementById('coins').textContent = coinCount;
    }
    
    function updateHealthUI() {
        const healthPercent = (player.health / player.maxHealth) * 100;
        document.getElementById('health-fill').style.width = `${healthPercent}%`;
    }
    
    function showGameOver() {
        gameRunning = false;
        document.getElementById('final-score').textContent = score;
        showScreen('game-over-screen');
    }
    
    function showWin() {
        gameRunning = false;
        document.getElementById('win-score').textContent = score;
        showScreen('win-screen');
        AudioSystem.playWin();
    }
    
    // ============================================
    // GAME FUNCTIONS
    // ============================================
    function startGame() {
        AudioSystem.init();
        resetGame();
        gameRunning = true;
        showScreen(null);
        lastTime = performance.now();
        requestAnimationFrame(gameLoop);
    }
    
    function restartGame() {
        resetGame();
        gameRunning = true;
        showScreen(null);
        lastTime = performance.now();
        requestAnimationFrame(gameLoop);
    }
    
    function resetGame() {
        player = new Player(100, 400);
        tiles = JSON.parse(JSON.stringify(levelData.tiles));
        enemies = levelData.enemies.map(e => new Enemy(e.x, e.y, e.type));
        coins = levelData.coins.map(c => new Coin(c.x, c.y));
        particles = [];
        score = 0;
        coinCount = 0;
        shakeAmount = 0;
        updateScoreUI();
        updateCoinsUI();
        updateHealthUI();
    }
    
    function gameOver() {
        showGameOver();
    }
    
    function gameLoop(timestamp) {
        if (!gameRunning) return;
        
        const dt = Math.min((timestamp - lastTime) / 1000, 0.05);
        lastTime = timestamp;
        
        update(dt);
        render();
        
        requestAnimationFrame(gameLoop);
    }
    
    function update(dt) {
        const input = new InputHandler();
        
        // Reset input each frame since we're creating new instance
        // Actually, we need to preserve state - let's refactor
        
        player.update(dt, input, tiles, enemies);
        
        // Update enemies
        enemies.forEach(enemy => enemy.update(dt, tiles));
        
        // Update coins
        coins.forEach(coin => {
            coin.update(dt);
            
            // Check collection
            if (!coin.collected &&
                player.x < coin.x + coin.width &&
                player.x + player.width > coin.x &&
                player.y < coin.y + coin.height &&
                player.y + player.height > coin.y) {
                
                coin.collected = true;
                coinCount++;
                score += 50;
                AudioSystem.playCoin();
                updateCoinsUI();
                updateScoreUI();
            }
        });
        
        // Update particles
        particles = particles.filter(p => !p.isDead());
        particles.forEach(p => p.update(dt));
        
        // Screen shake decay
        if (shakeAmount > 0) {
            shakeAmount -= dt * 30;
            if (shakeAmount < 0) shakeAmount = 0;
        }
        
        // Win condition (collect all coins)
        if (coinCount >= coins.length) {
            showWin();
        }
    }
    
    function render() {
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Apply screen shake
        const shakeX = shakeAmount ? (Math.random() - 0.5) * shakeAmount * 2 : 0;
        const shakeY = shakeAmount ? (Math.random() - 0.5) * shakeAmount * 2 : 0;
        
        ctx.save();
        ctx.translate(shakeX, shakeY);
        
        // Draw tiles
        tiles.forEach(tile => {
            if (tile.type === 'solid') {
                ctx.fillStyle = '#5D4037';
                ctx.fillRect(tile.x, tile.y, tile.width, tile.height);
                
                // Detail
                ctx.fillStyle = '#4E342E';
                ctx.fillRect(tile.x + 2, tile.y + 2, tile.width - 4, tile.height - 4);
            }
        });
        
        // Draw coins
        coins.forEach(coin => coin.draw(ctx));
        
        // Draw enemies
        enemies.forEach(enemy => enemy.draw(ctx));
        
        // Draw player
        player.draw(ctx);
        
        // Draw particles
        particles.forEach(p => p.draw(ctx));
        
        ctx.restore();
    }
    
    function triggerShake(amount) {
        shakeAmount = Math.max(shakeAmount, amount);
    }
    
    // ============================================
    // INITIALIZATION
    // ============================================
    window.onload = () => {
        initUI();
    };
    
})();