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
      logs: [],
      suctionOn: false,
      rosConnected: true,
      cameraConnected: true,
      micActive: false,
      positionHistory: [],
      socket: null,
      isSending: false
    };
  },
  mounted() {
    this.connectWebSocket();
  },
  methods: {
    async sendToGPT(message) {
      if (this.isSending) {
        this.logs.push("[gpt] 요청 중입니다. 잠시 기다리세요.");
        return;
      }

      this.isSending = true;
      try {
        const response = await fetch("https://api.openai.com/v1/chat/completions", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${import.meta.env.VITE_OPENAI_API_KEY}`
          },
          body: JSON.stringify({
            model: "gpt-4o-mini",
            messages: [
              {
                role: "system",
                content: "사용자의 자연어 명령을 '1', '2', ..., 또는 'z_up', 'suction_on' 등으로 반환하세요. 여러 동작이 필요한 경우 쉼표(,)로 구분하여 반환하세요. 예: '2,2,2'"
              },
              { role: "user", content: message }
            ]
          })
        });

        if (!response.ok) {
          this.logs.push(`[gpt] 요청 실패: ${response.status}`);
          return;
        }

        const data = await response.json();
        const rawCommand = data?.choices?.[0]?.message?.content?.trim();

        if (!rawCommand) {
          this.logs.push("[gpt] 응답이 비어 있습니다.");
          return;
        }

        this.gptResponse = rawCommand;
        this.logs.push(`[GPT] ${rawCommand}`);

        const commands = rawCommand.split(',').map(c => c.trim());
        const COMMAND_MAP = {
          z_up: "2",
          z_down: "1",
          suction_on: "4",
          suction_off: "3",
          reset: "7"
        };

        for (const cmd of commands) {
          await new Promise(resolve => setTimeout(resolve, 100)); // 100ms 딜레이

          const resolved = COMMAND_MAP[cmd] || cmd;

          if (['1','2','3','4','5','6','7','8','9','10'].includes(resolved)) {
            this.sendCommandToROS(resolved, "gpt");
          } else {
            this.logs.push(`[gpt] 인식되지 않은 명령어: ${cmd}`);
          }
        }

      } catch (error) {
        console.error("[gpt] 오류:", error);
        this.logs.push("[gpt] 요청 중 오류 발생");
      } finally {
        this.isSending = false;
      }
    },

    connectWebSocket() {
      this.socket = new WebSocket("ws://192.168.110.121:20000/");

      this.socket.onopen = () => {
        this.logs.push("[ws] 웹소켓 연결됨");
        this.socket.send(JSON.stringify({ type: "init", message: "dashboard connected" }));
      };

      this.socket.onmessage = (event) => {
        const text = event.data.trim();
        console.log("[WS Raw] Received:", text);

        try {
          // 좌표 데이터인 경우
          const data = JSON.parse(text);
          if (typeof data.x === "number" && typeof data.y === "number") {
            this.positionHistory.push({ x: data.x, y: data.y });
            if (this.positionHistory.length > 30) this.positionHistory.shift();
            this.logs.push(`[ws] (${data.x.toFixed(1)}, ${data.y.toFixed(1)})`);
            return;
          }
        } catch {
          // JSON이 아니면 명령일 가능성 있음
        }

        // 제스처 명령인지 판단
        if (/^\d+$/.test(text) && ['1','2','3','4','5','6','7','8','9','10'].includes(text)) {
          this.logs.push(`[gesture] 제스처 명령 수신: ${text}`);
        } else {
          this.logs.push(`[ws] 수신 문자열: ${text}`);
        }
      };

      this.socket.onerror = (err) => {
        console.error("[ws] 에러 발생:", err);
        this.logs.push("[ws] 연결 에러");
      };

      this.socket.onclose = () => {
        this.logs.push("[ws] 연결 종료됨");
      };
    },


    sendCommandToROS(code, source = "button") {
      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        this.socket.send(code);
        this.logs.push(`[${source}] 명령 전송: ${code}`);
      } else {
        this.logs.push("[error] WebSocket 연결 안됨");
      }
    },

    moveZUp() {
      this.logs.push("[button] Z축 위로 이동");
      this.sendCommandToROS("2");
    },

    moveZDown() {
      this.logs.push("[button] Z축 아래로 이동");
      this.sendCommandToROS("1");
    },

    toggleSuction(force = null) {
      const isForce = typeof force === 'boolean';
      const newState = isForce ? force : !this.suctionOn;

      this.suctionOn = newState;
      this.logs.push(`[button] 석션 ${this.suctionOn ? 'ON' : 'OFF'}`);
      this.sendCommandToROS(this.suctionOn ? "4" : "3");
    },
    

    resetDobot() {
      this.logs.push("[button] 두봇 초기화");
      this.sendCommandToROS("7");
      this.logs = [];
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
    }
  },
  computed: {
    commandSourceStats() {
      return [
        this.logs.filter((l) => l.includes('[voice]')).length,
        this.logs.filter((l) => l.includes('[button]')).length
      ];
    }
  }
};
</script>

<template>
  <div class="dashboard-wrapper">
    <header class="card header">
      <span>두봇 제어 대시보드</span>
      <div class="status-indicators">
        <span :class="rosConnected ? 'status-ok' : 'status-bad'">ROS</span>
        <span :class="cameraConnected ? 'status-ok' : 'status-bad'">CAM</span>
        <span :class="micActive ? 'status-ok' : 'status-bad'">MIC</span>
      </div>
    </header>

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

    <div class="card control-panel">
      <button @click="moveZUp">⬆ Z UP</button>
      <button @click="moveZDown">⬇ Z DOWN</button>
      <button @click="toggleSuction">
        {{ suctionOn ? '❌ 석션 OFF' : '🧲 석션 ON' }}
      </button>
      <button @click="resetDobot">🔄 리셋</button>
    </div>

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

    <div class="card log-section">
      <h3>📄 실시간 로그</h3>
      <ul>
        <li v-for="(entry, index) in logs.slice(-5).reverse()" :key="index">{{ entry }}</li>
      </ul>
    </div>
  </div>
</template>

<style src="./Dashboard.css"></style>
