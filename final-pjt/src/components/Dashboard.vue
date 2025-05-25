<script>
import './Dashboard.css'
import DoughnutChart from './DoughnutChart.vue'
import ScatterChart from './ScatterChart.vue'

export default {
  components: { DoughnutChart, ScatterChart },
  data() {
    return {
      voiceText: '',
      gptResponse: '',
      logs: [
        '[voice] 시작',
        '[gesture] Z축 위로 이동',
        '[button] 석션 ON',
        '[voice] 흡착 시작',
        '[gesture] Z축 아래로 이동',
        '[button] 리셋',
      ],
      suctionOn: false,
      rosConnected: true,
      cameraConnected: true,
      micActive: false,
      positionHistory: [
        { x: 100, y: 120, z: 50 },
        { x: 110, y: 125, z: 80 },
        { x: 115, y: 130, z: 100 },
      ],
    }
  },
  methods: {
    async sendToGPT(message) {
      const response = await fetch('http://localhost:4000/api/gpt', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message }),
      })

      const data = await response.json()
      const command = data.choices[0].message.content.trim()
      this.gptResponse = command
      this.logs.push(`[GPT] ${command}`)

      if (command === 'suction_on') this.suctionOn = true
      if (command === 'suction_off') this.suctionOn = false
      if (command === 'z_up') this.moveZUp()
      if (command === 'z_down') this.moveZDown()
      if (command === 'reset') this.resetDobot()
    },

    startVoiceRecognition() {
  this.voiceText = '🎤 듣는 중...';

  this.recognition = new (window.SpeechRecognition || window.webkitSpeechRecognition)();
  this.recognition.lang = 'ko-KR';
  this.recognition.interimResults = false;
  this.recognition.maxAlternatives = 1;

  this.recognition.start();

  this.recognition.onresult = (event) => {
    const transcript = event.results[0][0].transcript;
    this.voiceText = transcript;
    this.logs.push(`[voice] ${transcript}`);
    this.sendToGPT(transcript);
  };

  this.recognition.onerror = (event) => {
    this.voiceText = '음성 인식 실패: ' + event.error;
    this.logs.push(`[voice] 음성 인식 에러 (${event.error})`);
  };

  this.recognition.onend = () => {
    this.micActive = false;
  };

  this.micActive = true;
},
stopVoiceRecognition() {
  if (this.recognition) {
    this.recognition.stop();
    this.voiceText = '🛑 인식 중지됨';
    this.logs.push(`[voice] 음성 인식 중단`);
    this.micActive = false;
  }
},

    moveZUp() {
      this.logs.push(`[gesture] Z축 위로 이동`)
    },
    moveZDown() {
      this.logs.push(`[gesture] Z축 아래로 이동`)
    },
    toggleSuction() {
      this.suctionOn = !this.suctionOn
      this.logs.push(`[button] 석션 ${this.suctionOn ? 'ON' : 'OFF'}`)
    },
    resetDobot() {
      this.logs.push(`[button] 도봇 초기화 명령 전송`)
    },
  },
  computed: {
    commandSourceStats() {
      return [
        this.logs.filter((l) => l.includes('[voice]')).length,
        this.logs.filter((l) => l.includes('[gesture]')).length,
        this.logs.filter((l) => l.includes('[button]')).length,
      ]
    },
  },
}
</script>

<template>
  <div class="dashboard-wrapper">
    <!-- 헤더 -->
    <header class="card header">
      <span>두봇 제어 대시보드</span>
      <div class="status-indicators">
        <span :class="rosConnected ? 'status-ok' : 'status-bad'">ROS</span>
        <span :class="cameraConnected ? 'status-ok' : 'status-bad'">CAM</span>
        <span :class="micActive ? 'status-ok' : 'status-bad'">MIC</span>
      </div>
    </header>

    <!-- 상단 차트 두 개 -->
    <div class="dashboard-row">
      <div class="card chart-box">
        <h2>로봇 이동 위치</h2>
        <ScatterChart :points="positionHistory" />
      </div>
      <div class="card chart-box">
        <h2>입력 방식 비율</h2>
        <DoughnutChart :data="commandSourceStats" />
      </div>
    </div>

    <!-- 제어 버튼 -->
    <div class="card control-panel">
      <button @click="moveZUp">⬆ Z UP</button>
      <button @click="moveZDown">⬇ Z DOWN</button>
      <button @click="toggleSuction">
        {{ suctionOn ? '❌ 석션 OFF' : '🧲 석션 ON' }}
      </button>
      <button @click="resetDobot">🔄 리셋</button>
    </div>

    <!-- 음성 / GPT -->
    <div class="dashboard-row">
      <div class="card voice-section">
        <h3>음성 명령</h3>
        <button v-if="!micActive" @click="startVoiceRecognition">🎤 음성 인식 시작</button>
        <button v-else @click="stopVoiceRecognition">🛑 음성 인식 종료</button>
        <p>{{ voiceText }}</p>
      </div>
      <div class="card gpt-section">
        <h3>GPT 응답</h3>
        <p>{{ gptResponse }}</p>
      </div>
    </div>

    <!-- 실시간 로그 -->
    <div class="card log-section">
      <h3>📄 실시간 로그</h3>
      <ul>
        <li v-for="(entry, index) in logs.slice(-5).reverse()" :key="index">{{ entry }}</li>
      </ul>
    </div>
  </div>
</template>

<style src="./Dashboard.css"></style>
