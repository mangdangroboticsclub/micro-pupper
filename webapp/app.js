/**
 * MicroPupper Gait Animator - Web Bluetooth Control
 * 
 * Connects to MicroPupper via BLE and provides animation tools
 * for developing gait patterns.
 */

// ═══════════════════════════════════════════════════════
// BLE CONFIGURATION
// ═══════════════════════════════════════════════════════

const BLE_SERVICE_UUID = '0d9be2a0-4757-43d9-83df-704ae274b8df';
const BLE_CHARACTERISTIC_UUID = '8116d8c0-d45d-4fdf-998e-33ab8c471d59';
const BLE_DEVICE_NAME = 'MicroPupper';

// ═══════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════

let bleDevice = null;
let bleCharacteristic = null;
let isConnected = false;
let keyframes = [];
let isPlaying = false;
let keyframeMode = 'unified'; // 'unified' or 'offset'

// Default stance positions
const STANCE = {
    fr: 90,
    fl: 90,
    br: 270,
    bl: 270,
    speed: 1000,
    delay: 200
};

// Default leg offsets for diagonal gait (FL-BR, FR-BL diagonal pairs)
// FL starts first, BR follows 30ms later, FR starts 100ms later, BL follows 30ms after FR
let legOffsets = {
    fl: 0,
    br: 30,
    fr: 100,
    bl: 130
};

// ═══════════════════════════════════════════════════════
// DOM ELEMENTS
// ═══════════════════════════════════════════════════════

const elements = {
    // Connection
    connectBtn: document.getElementById('connect-btn'),
    disconnectBtn: document.getElementById('disconnect-btn'),
    statusDot: document.querySelector('.status-dot'),
    statusText: document.getElementById('status-text'),
    
    // Quick Controls
    stanceBtn: document.getElementById('stance-btn'),
    pingBtn: document.getElementById('ping-btn'),
    
    // Mode Toggle
    modeUnifiedBtn: document.getElementById('mode-unified-btn'),
    modeOffsetBtn: document.getElementById('mode-offset-btn'),
    unifiedTiming: document.getElementById('unified-timing'),
    offsetSettings: document.getElementById('offset-settings'),
    delayLabel: document.getElementById('delay-label'),
    
    // Offset inputs
    offsetFl: document.getElementById('offset-fl'),
    offsetBr: document.getElementById('offset-br'),
    offsetFr: document.getElementById('offset-fr'),
    offsetBl: document.getElementById('offset-bl'),
    
    // Servo Controls
    frSlider: document.getElementById('fr-slider'),
    flSlider: document.getElementById('fl-slider'),
    brSlider: document.getElementById('br-slider'),
    blSlider: document.getElementById('bl-slider'),
    frValue: document.getElementById('fr-value'),
    flValue: document.getElementById('fl-value'),
    brValue: document.getElementById('br-value'),
    blValue: document.getElementById('bl-value'),
    speedInput: document.getElementById('speed-input'),
    delayInput: document.getElementById('delay-input'),
    
    sendBtn: document.getElementById('send-btn'),
    addKeyframeBtn: document.getElementById('add-keyframe-btn'),
    
    // Animation
    playBtn: document.getElementById('play-btn'),
    cycleCount: document.getElementById('cycle-count'),
    clearBtn: document.getElementById('clear-btn'),
    timeline: document.getElementById('timeline'),
    animationName: document.getElementById('animation-name'),
    saveBtn: document.getElementById('save-btn'),
    addBtn: document.getElementById('add-btn'),
    savedList: document.getElementById('saved-list'),
    
    // Import/Export
    importSingleBtn: document.getElementById('import-single-btn'),
    importBatchBtn: document.getElementById('import-batch-btn'),
    exportAllBtn: document.getElementById('export-all-btn'),
    exportSelectedBtn: document.getElementById('export-selected-btn'),
    importFileInput: document.getElementById('import-file-input'),
    importBatchInput: document.getElementById('import-batch-input'),
    
    // Log
    log: document.getElementById('log')
};

// Selected animations for export
let selectedAnimations = new Set();

// ═══════════════════════════════════════════════════════
// LOGGING
// ═══════════════════════════════════════════════════════

function log(message, type = 'info') {
    const time = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.innerHTML = `<span class="log-time">${time}</span>${message}`;
    elements.log.insertBefore(entry, elements.log.firstChild);
    
    // Keep only last 50 entries
    while (elements.log.children.length > 50) {
        elements.log.removeChild(elements.log.lastChild);
    }
    
    console.log(`[${type}] ${message}`);
}

// ═══════════════════════════════════════════════════════
// BLUETOOTH CONNECTION
// ═══════════════════════════════════════════════════════

async function connect() {
    try {
        log('Requesting BLE device...');
        
        bleDevice = await navigator.bluetooth.requestDevice({
            filters: [{ name: BLE_DEVICE_NAME }],
            optionalServices: [BLE_SERVICE_UUID]
        });
        
        log(`Found device: ${bleDevice.name}`);
        
        bleDevice.addEventListener('gattserverdisconnected', onDisconnected);
        
        log('Connecting to GATT server...');
        const server = await bleDevice.gatt.connect();
        
        log('Getting service...');
        const service = await server.getPrimaryService(BLE_SERVICE_UUID);
        
        log('Getting characteristic...');
        bleCharacteristic = await service.getCharacteristic(BLE_CHARACTERISTIC_UUID);
        
        // Subscribe to notifications
        await bleCharacteristic.startNotifications();
        bleCharacteristic.addEventListener('characteristicvaluechanged', onNotification);
        
        setConnected(true);
        log('Connected successfully!', 'success');
        
    } catch (error) {
        log(`Connection failed: ${error.message}`, 'error');
        setConnected(false);
    }
}

function disconnect() {
    if (bleDevice && bleDevice.gatt.connected) {
        bleDevice.gatt.disconnect();
    }
    setConnected(false);
    log('Disconnected', 'info');
}

function onDisconnected() {
    setConnected(false);
    log('Device disconnected', 'error');
}

function onNotification(event) {
    const value = new TextDecoder().decode(event.target.value);
    log(`Received: ${value}`, 'info');
}

function setConnected(connected) {
    isConnected = connected;
    elements.statusDot.className = `status-dot ${connected ? 'connected' : 'disconnected'}`;
    elements.statusText.textContent = connected ? 'Connected' : 'Disconnected';
    elements.connectBtn.disabled = connected;
    elements.disconnectBtn.disabled = !connected;
}

// ═══════════════════════════════════════════════════════
// BLE COMMANDS
// ═══════════════════════════════════════════════════════

// BLE chunk size - conservative to work with default MTU
const BLE_CHUNK_SIZE = 180;  // Max payload per chunk (leaves room for chunk header)
const BLE_CHUNK_DELAY = 50;  // ms between chunks for reliability

/**
 * Send raw data to BLE characteristic
 */
async function sendRaw(data) {
    if (!isConnected || !bleCharacteristic) {
        log('Not connected!', 'error');
        return false;
    }
    
    try {
        const encoder = new TextEncoder();
        await bleCharacteristic.writeValue(encoder.encode(data));
        return true;
    } catch (error) {
        log(`Send failed: ${error.message}`, 'error');
        return false;
    }
}

/**
 * Send command - automatically chunks if message is too large
 * Chunk format: {"k":<num>,"t":<total>,"d":"<data>"}
 */
async function sendCommand(cmd) {
    if (!isConnected || !bleCharacteristic) {
        log('Not connected!', 'error');
        return false;
    }
    
    const json = JSON.stringify(cmd);
    
    // If small enough, send directly
    if (json.length <= BLE_CHUNK_SIZE) {
        const success = await sendRaw(json);
        if (success) log(`Sent: ${json}`, 'success');
        return success;
    }
    
    // Need to chunk the message
    const totalChunks = Math.ceil(json.length / BLE_CHUNK_SIZE);
    log(`Chunking: ${json.length} bytes into ${totalChunks} chunks...`, 'info');
    
    for (let i = 0; i < totalChunks; i++) {
        const start = i * BLE_CHUNK_SIZE;
        const end = Math.min(start + BLE_CHUNK_SIZE, json.length);
        const payload = json.slice(start, end);
        
        const chunk = JSON.stringify({
            k: i + 1,           // chunk number (1-based)
            t: totalChunks,     // total chunks
            d: payload          // data payload
        });
        
        const success = await sendRaw(chunk);
        if (!success) {
            log(`Chunk ${i + 1}/${totalChunks} failed`, 'error');
            return false;
        }
        
        log(`Chunk ${i + 1}/${totalChunks} sent`, 'info');
        
        // Wait between chunks for reliability
        if (i < totalChunks - 1) {
            await sleep(BLE_CHUNK_DELAY);
        }
    }
    
    log(`All ${totalChunks} chunks sent successfully`, 'success');
    return true;
}

/**
 * Send a single servo position command
 * Format: {"s":[fr,fl,br,bl,speed,delay]}
 */
async function sendServoPosition(fr, fl, br, bl, speed, delay) {
    return sendCommand({
        s: [fr, fl, br, bl, speed, delay]
    });
}

/**
 * Send multiple moves as a sequence
 * Format: {"m":[[fr,fl,br,bl,speed,delay],[...]]}
 */
async function sendServoSequence(moves) {
    return sendCommand({
        m: moves.map(m => [m.fr, m.fl, m.br, m.bl, m.speed, m.delay])
    });
}

/**
 * Send ping command
 * Format: {"p":1}
 */
async function sendPing() {
    return sendCommand({ p: 1 });
}

/**
 * Send stance (return to default) command
 * Format: {"r":1}
 */
async function sendStance() {
    return sendCommand({ r: 1 });
}

/**
 * Send offset gait animation
 * Format: {"o":{"d":[fl,br,fr,bl],"s":speed,"k":[[fr,fl,br,bl,delay],...]}}
 * 
 * The device will execute each keyframe with staggered timing based on offsets.
 * Each leg starts its keyframe sequence at its offset time.
 * Each keyframe includes the step delay (how long to hold that position).
 */
async function sendOffsetGait(offsets, keyframes, speed) {
    return sendCommand({
        o: {
            d: [offsets.fl, offsets.br, offsets.fr, offsets.bl],  // leg start offsets in order FL,BR,FR,BL
            s: speed,
            k: keyframes.map(kf => [kf.fr, kf.fl, kf.br, kf.bl, kf.delay || 200])  // angle arrays + delay
        }
    });
}

/**
 * Send interleaved movement sequence for offset gait
 * This pre-calculates the exact timing and sends individual moves
 * Format: {"i":[[time,leg,angle,speed],...]}
 */
async function sendInterleavedSequence(moves) {
    return sendCommand({
        i: moves
    });
}

// ═══════════════════════════════════════════════════════
// SERVO CONTROL
// ═══════════════════════════════════════════════════════

function getCurrentPosition() {
    return {
        fr: parseInt(elements.frValue.value) || 90,
        fl: parseInt(elements.flValue.value) || 90,
        br: parseInt(elements.brValue.value) || 270,
        bl: parseInt(elements.blValue.value) || 270,
        speed: parseInt(elements.speedInput.value) || 1000,
        delay: parseInt(elements.delayInput.value) || 200
    };
}

/**
 * Get current leg offsets from UI
 */
function getCurrentOffsets() {
    return {
        fl: parseInt(elements.offsetFl?.value) || 0,
        br: parseInt(elements.offsetBr?.value) || 30,
        fr: parseInt(elements.offsetFr?.value) || 100,
        bl: parseInt(elements.offsetBl?.value) || 100
    };
}

/**
 * Apply offset preset
 * 
 * Diagonal gait pairing (matching KTurtle):
 *   Pair 1: FL + BR (front-left and back-right) - start together
 *   Pair 2: FR + BL (front-right and back-left) - start after offset
 */
function applyOffsetPreset(preset) {
    const presets = {
        // Standard diagonal: Pair 1 at 0ms, Pair 2 at 100ms
        'diagonal': { fl: 0, br: 0, fr: 100, bl: 100 },
        // Staggered: slight delay within pairs for smoother motion
        'staggered': { fl: 0, br: 30, fr: 100, bl: 130 },
        // Wave: sequential leg movement
        'wave': { fl: 0, fr: 75, br: 150, bl: 225 }
    };
    
    const values = presets[preset];
    if (values) {
        elements.offsetFl.value = values.fl;
        elements.offsetBr.value = values.br;
        elements.offsetFr.value = values.fr;
        elements.offsetBl.value = values.bl;
        legOffsets = values;
        log(`Applied offset preset: ${preset}`, 'success');
    }
}

function setPosition(pos) {
    elements.frSlider.value = elements.frValue.value = pos.fr;
    elements.flSlider.value = elements.flValue.value = pos.fl;
    elements.brSlider.value = elements.brValue.value = pos.br;
    elements.blSlider.value = elements.blValue.value = pos.bl;
    if (pos.speed !== undefined) elements.speedInput.value = pos.speed;
    if (pos.delay !== undefined) elements.delayInput.value = pos.delay;
}

/**
 * Set the keyframe mode (unified or offset)
 */
function setKeyframeMode(mode) {
    keyframeMode = mode;
    
    // Update button styles
    elements.modeUnifiedBtn.classList.toggle('active', mode === 'unified');
    elements.modeOffsetBtn.classList.toggle('active', mode === 'offset');
    
    // Show/hide offset settings
    if (elements.offsetSettings) {
        elements.offsetSettings.style.display = mode === 'offset' ? 'block' : 'none';
    }
    
    // Update delay label based on mode
    if (elements.delayLabel) {
        elements.delayLabel.textContent = mode === 'offset' ? 'Step Duration (ms)' : 'Delay (ms)';
    }
    
    log(`Keyframe mode: ${mode === 'unified' ? 'Unified (all legs same timing)' : 'Offset Gait (staggered diagonal)'}`, 'info');
}

function syncSliderToValue(sliderId, valueId) {
    const slider = document.getElementById(sliderId);
    const value = document.getElementById(valueId);
    
    slider.addEventListener('input', () => value.value = slider.value);
    value.addEventListener('input', () => slider.value = value.value);
}

// ═══════════════════════════════════════════════════════
// KEYFRAMES & TIMELINE
// ═══════════════════════════════════════════════════════

// Color palette for keyframe segments
const SEGMENT_COLORS = [
    '#4a90d9', '#5cb85c', '#f0ad4e', '#d9534f', '#5bc0de',
    '#9b59b6', '#e74c3c', '#1abc9c', '#f39c12', '#3498db'
];

function addKeyframe() {
    const pos = getCurrentPosition();
    keyframes.push(pos);
    renderTimeline();
    renderTimingBar();
    log(`Added keyframe ${keyframes.length}: FR=${pos.fr} FL=${pos.fl} BR=${pos.br} BL=${pos.bl}`, 'success');
}

function removeKeyframe(index) {
    keyframes.splice(index, 1);
    renderTimeline();
    renderTimingBar();
    log(`Removed keyframe ${index + 1}`, 'info');
}

function clearKeyframes() {
    keyframes = [];
    renderTimeline();
    renderTimingBar();
    log('Cleared all keyframes', 'info');
}

function moveKeyframe(index, direction) {
    const newIndex = index + direction;
    if (newIndex < 0 || newIndex >= keyframes.length) return;
    
    // Swap keyframes
    [keyframes[index], keyframes[newIndex]] = [keyframes[newIndex], keyframes[index]];
    renderTimeline();
    renderTimingBar();
    log(`Moved keyframe ${index + 1} ${direction > 0 ? 'down' : 'up'}`, 'info');
}

function updateKeyframe(index, field, value) {
    if (index < 0 || index >= keyframes.length) return;
    
    const numValue = parseInt(value) || 0;
    keyframes[index][field] = numValue;
    renderTimingBar(); // Update timing visualization
}

function renderTimeline() {
    if (keyframes.length === 0) {
        elements.timeline.innerHTML = '<div class="empty-message">No keyframes yet. Add positions above!</div>';
        return;
    }
    
    // In offset mode, show the current offsets at the top
    let offsetHeader = '';
    if (keyframeMode === 'offset') {
        const offsets = getCurrentOffsets();
        offsetHeader = `
            <div class="offset-timeline-header">
                <span>🦿 Offset Gait Mode</span>
                <span class="offset-summary">FL:${offsets.fl}ms → BR:${offsets.br}ms → FR:${offsets.fr}ms → BL:${offsets.bl}ms</span>
            </div>
        `;
    }
    
    elements.timeline.innerHTML = offsetHeader + keyframes.map((kf, i) => `
        <div class="keyframe" id="keyframe-${i}" style="border-left-color: ${SEGMENT_COLORS[i % SEGMENT_COLORS.length]}">
            <div class="keyframe-reorder">
                <button onclick="moveKeyframe(${i}, -1)" ${i === 0 ? 'disabled' : ''} title="Move up">▲</button>
                <button onclick="moveKeyframe(${i}, 1)" ${i === keyframes.length - 1 ? 'disabled' : ''} title="Move down">▼</button>
            </div>
            <span class="keyframe-number">#${i + 1}</span>
            <div class="keyframe-inputs">
                <div class="keyframe-field">
                    <label>FR</label>
                    <input type="number" value="${kf.fr}" min="0" max="360" 
                           onchange="updateKeyframe(${i}, 'fr', this.value)">
                </div>
                <div class="keyframe-field">
                    <label>FL</label>
                    <input type="number" value="${kf.fl}" min="0" max="360"
                           onchange="updateKeyframe(${i}, 'fl', this.value)">
                </div>
                <div class="keyframe-field">
                    <label>BR</label>
                    <input type="number" value="${kf.br}" min="0" max="360"
                           onchange="updateKeyframe(${i}, 'br', this.value)">
                </div>
                <div class="keyframe-field">
                    <label>BL</label>
                    <input type="number" value="${kf.bl}" min="0" max="360"
                           onchange="updateKeyframe(${i}, 'bl', this.value)">
                </div>
                <div class="keyframe-field speed">
                    <label>Spd</label>
                    <input type="number" value="${kf.speed}" min="0" max="4095"
                           onchange="updateKeyframe(${i}, 'speed', this.value)">
                </div>
                <div class="keyframe-field delay">
                    <label>${keyframeMode === 'offset' ? 'Dur' : 'Dly'}</label>
                    <input type="number" value="${kf.delay}" min="0" max="2000"
                           onchange="updateKeyframe(${i}, 'delay', this.value)">
                </div>
            </div>
            <div class="keyframe-actions">
                <button class="keyframe-delete" onclick="removeKeyframe(${i})" title="Delete">×</button>
            </div>
        </div>
    `).join('');
}

function renderTimingBar() {
    const timingBar = document.getElementById('timing-bar');
    const totalTimeLabel = document.getElementById('total-time');
    
    if (keyframes.length === 0) {
        timingBar.innerHTML = '';
        totalTimeLabel.textContent = '0ms';
        return;
    }
    
    // For offset mode, calculate total animation time including offsets
    if (keyframeMode === 'offset') {
        const offsets = getCurrentOffsets();
        const maxOffset = Math.max(offsets.fl, offsets.br, offsets.fr, offsets.bl);
        const keyframeDurations = keyframes.reduce((sum, kf) => sum + (kf.delay || 0), 0);
        const totalTime = maxOffset + keyframeDurations;
        totalTimeLabel.textContent = `${totalTime}ms (+ ${maxOffset}ms offset)`;
    } else {
        const totalTime = keyframes.reduce((sum, kf) => sum + (kf.delay || 0), 0);
        totalTimeLabel.textContent = `${totalTime}ms`;
    }
    
    const totalDelay = keyframes.reduce((sum, kf) => sum + (kf.delay || 0), 0);
    
    if (totalDelay === 0) {
        timingBar.innerHTML = keyframes.map((kf, i) => 
            `<div class="timing-segment" style="flex: 1; background: ${SEGMENT_COLORS[i % SEGMENT_COLORS.length]}">#${i + 1}</div>`
        ).join('');
        return;
    }
    
    timingBar.innerHTML = keyframes.map((kf, i) => {
        const delay = kf.delay || 0;
        return `<div class="timing-segment" id="timing-segment-${i}"
                     style="flex: ${delay}; background: ${SEGMENT_COLORS[i % SEGMENT_COLORS.length]}"
                     onclick="scrollToKeyframe(${i})"
                     title="Keyframe ${i + 1}: ${delay}ms">
                    #${i + 1} (${delay}ms)
                </div>`;
    }).join('');
}

function scrollToKeyframe(index) {
    const el = document.getElementById(`keyframe-${index}`);
    if (el) {
        el.scrollIntoView({ behavior: 'smooth', block: 'center' });
        el.style.animation = 'none';
        el.offsetHeight; // Trigger reflow
        el.style.animation = 'highlight-pulse 0.5s ease-out';
    }
}

function highlightKeyframe(index) {
    document.querySelectorAll('.keyframe').forEach((el, i) => {
        el.classList.toggle('playing', i === index);
    });
    document.querySelectorAll('.timing-segment').forEach((el, i) => {
        el.classList.toggle('playing', i === index);
    });
}

// ═══════════════════════════════════════════════════════
// ANIMATION PLAYBACK
// ═══════════════════════════════════════════════════════

/**
 * Build interleaved movement sequence for offset gait
 * 
 * Given keyframes and leg offsets, creates a sorted list of individual movements
 * with absolute timestamps. This allows proper staggered execution.
 * 
 * @returns Array of [time_ms, leg_id, angle, speed]
 *   leg_id: 0=FR, 1=FL, 2=BR, 3=BL
 */
function buildInterleavedSequence(keyframes, offsets, defaultSpeed) {
    const movements = [];
    const legs = ['fr', 'fl', 'br', 'bl'];
    const legIds = { fr: 0, fl: 1, br: 2, bl: 3 };
    
    for (let legIdx = 0; legIdx < legs.length; legIdx++) {
        const leg = legs[legIdx];
        let time = offsets[leg];  // Start time for this leg
        
        for (let kfIdx = 0; kfIdx < keyframes.length; kfIdx++) {
            const kf = keyframes[kfIdx];
            const angle = kf[leg];
            const speed = kf.speed || defaultSpeed;
            const duration = kf.delay || 100;
            
            movements.push({
                time: time,
                leg: leg,
                legId: legIds[leg],
                angle: angle,
                speed: speed,
                kfIndex: kfIdx
            });
            
            time += duration;  // Move to next keyframe time for this leg
        }
    }
    
    // Sort by time, then by leg order (FL, BR, FR, BL for diagonal pattern)
    const legOrder = { fl: 0, br: 1, fr: 2, bl: 3 };
    movements.sort((a, b) => {
        if (a.time !== b.time) return a.time - b.time;
        return legOrder[a.leg] - legOrder[b.leg];
    });
    
    return movements;
}

async function playAnimation() {
    if (keyframes.length === 0) {
        log('No keyframes to play!', 'error');
        return;
    }
    
    if (isPlaying) {
        stopAnimation();
        return;
    }
    
    isPlaying = true;
    elements.playBtn.textContent = '⏹️ Stop';
    
    try {
        // Get cycle count
        const cycles = parseInt(elements.cycleCount.value) || 1;
        
        if (keyframeMode === 'offset') {
            // Offset Gait Mode - staggered leg execution
            const offsets = getCurrentOffsets();
            const speed = parseInt(elements.speedInput.value) || 1000;
            
            // Duplicate keyframes for multiple cycles
            const repeatedKeyframes = [];
            for (let c = 0; c < cycles; c++) {
                repeatedKeyframes.push(...keyframes);
            }
            
            log(`Playing offset gait ${cycles} cycle(s) (FL:${offsets.fl}ms BR:${offsets.br}ms FR:${offsets.fr}ms BL:${offsets.bl}ms)...`, 'info');
            
            // Build and send the interleaved sequence
            const movements = buildInterleavedSequence(repeatedKeyframes, offsets, speed);
            
            // Calculate total animation time
            const maxOffset = Math.max(offsets.fl, offsets.br, offsets.fr, offsets.bl);
            const totalDuration = repeatedKeyframes.reduce((sum, kf) => sum + (kf.delay || 100), 0);
            const totalTime = maxOffset + totalDuration;
            
            // Send the offset gait command with repeated keyframes
            const success = await sendOffsetGait(offsets, repeatedKeyframes, speed);
            
            if (!success) {
                log('Failed to send offset gait', 'error');
                return;
            }
            
            // Animate UI to show progress
            let lastKfIndex = -1;
            for (const move of movements) {
                if (!isPlaying) break;
                
                const originalIndex = move.kfIndex % keyframes.length;
                if (originalIndex !== lastKfIndex) {
                    highlightKeyframe(originalIndex);
                    setPosition(keyframes[originalIndex]);
                    lastKfIndex = originalIndex;
                }
                
                // Wait for next movement
                const nextMove = movements[movements.indexOf(move) + 1];
                if (nextMove) {
                    await sleep(nextMove.time - move.time);
                }
            }
            
            // Wait for the last movements to complete
            await sleep(100);
            
        } else {
            // Unified Mode - all legs move together
            // Duplicate keyframes for multiple cycles
            const repeatedKeyframes = [];
            for (let c = 0; c < cycles; c++) {
                repeatedKeyframes.push(...keyframes);
            }
            
            log(`Sending ${repeatedKeyframes.length} keyframes (${cycles} cycle(s))...`, 'info');
            
            const success = await sendServoSequence(repeatedKeyframes);
            
            if (!success) {
                log('Failed to send animation queue', 'error');
                return;
            }
            
            // Animate the UI to show progress
            for (let i = 0; i < repeatedKeyframes.length; i++) {
                if (!isPlaying) break;
                
                const kf = repeatedKeyframes[i];
                const originalIndex = i % keyframes.length;
                highlightKeyframe(originalIndex);
                setPosition(keyframes[originalIndex]);
                await sleep(kf.delay);
            }
        }
        
    } catch (error) {
        log(`Playback error: ${error.message}`, 'error');
    }
    
    stopAnimation();
}

function stopAnimation() {
    isPlaying = false;
    elements.playBtn.textContent = '▶️ Play';
    highlightKeyframe(-1);
    log('Animation stopped', 'info');
}



function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// ═══════════════════════════════════════════════════════
// SAVE/LOAD ANIMATIONS
// ═══════════════════════════════════════════════════════

function saveAnimation() {
    const name = elements.animationName.value.trim() || `Animation ${Date.now()}`;
    
    const saved = getSavedAnimations();
    saved[name] = keyframes;
    localStorage.setItem('micropupper_animations', JSON.stringify(saved));
    
    renderSavedAnimations();
    log(`Saved animation: ${name}`, 'success');
}

function loadAnimation(name) {
    const saved = getSavedAnimations();
    if (saved[name]) {
        keyframes = saved[name].map(kf => ({...kf})); // Deep copy
        renderTimeline();
        renderTimingBar();
        elements.animationName.value = name;
        log(`Loaded animation: ${name}`, 'success');
    }
}

function addAnimation(name) {
    const saved = getSavedAnimations();
    if (saved[name]) {
        // Deep copy and append to existing keyframes
        const newKeyframes = saved[name].map(kf => ({...kf}));
        keyframes.push(...newKeyframes);
        renderTimeline();
        renderTimingBar();
        log(`Added ${newKeyframes.length} keyframe(s) from: ${name}`, 'success');
    }
}

function deleteAnimation(name) {
    const saved = getSavedAnimations();
    delete saved[name];
    localStorage.setItem('micropupper_animations', JSON.stringify(saved));
    renderSavedAnimations();
    log(`Deleted animation: ${name}`, 'info');
}

function getSavedAnimations() {
    try {
        return JSON.parse(localStorage.getItem('micropupper_animations') || '{}');
    } catch {
        return {};
    }
}

function renderSavedAnimations() {
    const saved = getSavedAnimations();
    const names = Object.keys(saved);
    
    if (names.length === 0) {
        elements.savedList.innerHTML = '<div class="empty-message">No saved animations</div>';
        selectedAnimations.clear();
        elements.exportSelectedBtn.disabled = true;
        return;
    }
    
    elements.savedList.innerHTML = names.map(name => {
        const isSelected = selectedAnimations.has(name);
        return `
            <div class="saved-item ${isSelected ? 'selected' : ''}" data-name="${name}">
                <div class="saved-item-left">
                    <input type="checkbox" class="saved-item-checkbox" 
                           ${isSelected ? 'checked' : ''}
                           onchange="toggleAnimationSelection('${name}', this.checked)">
                    <span class="saved-item-name" onclick="loadAnimation('${name}')">${name} (${saved[name].length} keyframes)</span>
                </div>
                <div class="saved-item-actions">
                    <button class="btn btn-success" onclick="addAnimation('${name}')">➕ Add</button>
                    <button class="btn btn-secondary" onclick="exportAnimation('${name}')">📤</button>
                    <button class="btn btn-danger" onclick="deleteAnimation('${name}')">Delete</button>
                </div>
            </div>
        `;
    }).join('');
    
    // Update export selected button state
    elements.exportSelectedBtn.disabled = selectedAnimations.size === 0;
}

function toggleAnimationSelection(name, selected) {
    if (selected) {
        selectedAnimations.add(name);
    } else {
        selectedAnimations.delete(name);
    }
    
    // Update the selected class on the item
    const item = document.querySelector(`[data-name="${name}"]`);
    if (item) {
        item.classList.toggle('selected', selected);
    }
    
    // Update export selected button
    elements.exportSelectedBtn.disabled = selectedAnimations.size === 0;
}

// ═══════════════════════════════════════════════════════
// IMPORT/EXPORT
// ═══════════════════════════════════════════════════════

/**
 * Download a file with given content
 */
function downloadFile(filename, content) {
    const blob = new Blob([content], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

/**
 * Export a single animation by name
 */
function exportAnimation(name) {
    const saved = getSavedAnimations();
    if (!saved[name]) {
        log(`Animation "${name}" not found`, 'error');
        return;
    }
    
    const data = {
        name: name,
        keyframes: saved[name],
        exported: new Date().toISOString(),
        version: '1.0'
    };
    
    const filename = `${name.replace(/[^a-z0-9]/gi, '_')}.json`;
    downloadFile(filename, JSON.stringify(data, null, 2));
    log(`Exported: ${name}`, 'success');
}

/**
 * Export all animations
 */
function exportAllAnimations() {
    const saved = getSavedAnimations();
    const names = Object.keys(saved);
    
    if (names.length === 0) {
        log('No animations to export', 'error');
        return;
    }
    
    const data = {
        animations: names.map(name => ({
            name: name,
            keyframes: saved[name]
        })),
        exported: new Date().toISOString(),
        version: '1.0'
    };
    
    const filename = `micropupper_all_animations_${Date.now()}.json`;
    downloadFile(filename, JSON.stringify(data, null, 2));
    log(`Exported ${names.length} animations`, 'success');
}

/**
 * Export selected animations
 */
function exportSelectedAnimations() {
    if (selectedAnimations.size === 0) {
        log('No animations selected', 'error');
        return;
    }
    
    const saved = getSavedAnimations();
    const data = {
        animations: Array.from(selectedAnimations).map(name => ({
            name: name,
            keyframes: saved[name]
        })),
        exported: new Date().toISOString(),
        version: '1.0'
    };
    
    const filename = `micropupper_selected_animations_${Date.now()}.json`;
    downloadFile(filename, JSON.stringify(data, null, 2));
    log(`Exported ${selectedAnimations.size} animations`, 'success');
}

/**
 * Import animation(s) from file
 */
async function importAnimationFile(file) {
    try {
        const text = await file.text();
        const data = JSON.parse(text);
        
        const saved = getSavedAnimations();
        let imported = 0;
        
        // Single animation format
        if (data.name && data.keyframes) {
            let finalName = data.name;
            let suffix = 1;
            
            // Handle name conflicts
            while (saved[finalName]) {
                finalName = `${data.name}_${suffix}`;
                suffix++;
            }
            
            saved[finalName] = data.keyframes;
            imported = 1;
            log(`Imported: ${finalName}`, 'success');
        }
        // Multiple animations format
        else if (data.animations && Array.isArray(data.animations)) {
            for (const anim of data.animations) {
                if (anim.name && anim.keyframes) {
                    let finalName = anim.name;
                    let suffix = 1;
                    
                    // Handle name conflicts
                    while (saved[finalName]) {
                        finalName = `${anim.name}_${suffix}`;
                        suffix++;
                    }
                    
                    saved[finalName] = anim.keyframes;
                    imported++;
                }
            }
            log(`Imported ${imported} animations`, 'success');
        } else {
            log('Invalid animation file format', 'error');
            return;
        }
        
        localStorage.setItem('micropupper_animations', JSON.stringify(saved));
        renderSavedAnimations();
        
    } catch (error) {
        log(`Import failed: ${error.message}`, 'error');
    }
}

/**
 * Handle single file import
 */
function handleSingleImport() {
    elements.importFileInput.value = '';
    elements.importFileInput.onchange = (e) => {
        if (e.target.files.length > 0) {
            importAnimationFile(e.target.files[0]);
        }
    };
    elements.importFileInput.click();
}

/**
 * Handle batch import
 */
function handleBatchImport() {
    elements.importBatchInput.value = '';
    elements.importBatchInput.onchange = async (e) => {
        if (e.target.files.length > 0) {
            log(`Importing ${e.target.files.length} files...`, 'info');
            for (const file of e.target.files) {
                await importAnimationFile(file);
            }
        }
    };
    elements.importBatchInput.click();
}

// ═══════════════════════════════════════════════════════
// EVENT HANDLERS
// ═══════════════════════════════════════════════════════

function initEventListeners() {
    // Connection
    elements.connectBtn.addEventListener('click', connect);
    elements.disconnectBtn.addEventListener('click', disconnect);
    
    // Quick Controls
    elements.stanceBtn.addEventListener('click', () => {
        setPosition(STANCE);
        sendStance();
    });
    elements.pingBtn.addEventListener('click', () => sendPing());
    
    // Servo Controls
    syncSliderToValue('fr-slider', 'fr-value');
    syncSliderToValue('fl-slider', 'fl-value');
    syncSliderToValue('br-slider', 'br-value');
    syncSliderToValue('bl-slider', 'bl-value');
    
    elements.sendBtn.addEventListener('click', () => {
        const pos = getCurrentPosition();
        sendServoPosition(pos.fr, pos.fl, pos.br, pos.bl, pos.speed, pos.delay);
    });
    elements.addKeyframeBtn.addEventListener('click', addKeyframe);
    
    // Animation
    elements.playBtn.addEventListener('click', playAnimation);
    elements.clearBtn.addEventListener('click', clearKeyframes);
    elements.saveBtn.addEventListener('click', saveAnimation);
    elements.addBtn.addEventListener('click', () => {
        const name = elements.animationName.value.trim();
        if (name) addAnimation(name);
    });
    
    // Import/Export
    elements.importSingleBtn.addEventListener('click', handleSingleImport);
    elements.importBatchBtn.addEventListener('click', handleBatchImport);
    elements.exportAllBtn.addEventListener('click', exportAllAnimations);
    elements.exportSelectedBtn.addEventListener('click', exportSelectedAnimations);
}

// ═══════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════

function init() {
    // Check Web Bluetooth support
    if (!navigator.bluetooth) {
        log('Web Bluetooth is not supported in this browser!', 'error');
        elements.connectBtn.disabled = true;
        elements.connectBtn.textContent = '❌ Web Bluetooth Not Supported';
    }
    
    initEventListeners();
    setPosition(STANCE);
    renderTimeline();
    renderTimingBar();
    renderSavedAnimations();
    
    log('MicroPupper Gait Animator ready!', 'success');
}

// Make functions available globally for onclick handlers
window.removeKeyframe = removeKeyframe;
window.moveKeyframe = moveKeyframe;
window.updateKeyframe = updateKeyframe;
window.scrollToKeyframe = scrollToKeyframe;
window.loadAnimation = loadAnimation;
window.addAnimation = addAnimation;
window.deleteAnimation = deleteAnimation;
window.exportAnimation = exportAnimation;
window.toggleAnimationSelection = toggleAnimationSelection;
window.setKeyframeMode = setKeyframeMode;
window.applyOffsetPreset = applyOffsetPreset;

// Start
document.addEventListener('DOMContentLoaded', init);
