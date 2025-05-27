<template>
  <canvas ref="canvas" style="width: 100%; height: 100%"></canvas>
</template>

<script>
import { Chart, DoughnutController, ArcElement, Tooltip, Legend } from 'chart.js';
Chart.register(DoughnutController, ArcElement, Tooltip, Legend);

export default {
  props: ['data'],
  data() {
    return {
      chart: null
    };
  },
  mounted() {
    this.renderChart();
  },
  watch: {
    data() {
      this.renderChart(); // 데이터 변경 시 차트 다시 그림
    }
  },
  methods: {
    renderChart() {
      const canvas = this.$refs.canvas;
      if (!canvas || !this.data || this.data.length === 0) {
        return; // 렌더링 안전 체크
      }

      const ctx = canvas.getContext('2d');

      if (this.chart) {
        this.chart.destroy(); // 기존 차트 제거
      }

      this.chart = new Chart(ctx, {
        type: 'doughnut',
        data: {
          labels: ['음성', '버튼'],
          datasets: [{
            data: this.data,
            backgroundColor: ['#66f2c5', '#42a5f5'],
            borderWidth: 1
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          plugins: {
            legend: {
              position: 'bottom',
              labels: {
                color: '#eaf7ff',
                padding: 16
              }
            },
            tooltip: {
              callbacks: {
                label: (ctx) => {
                  const total = ctx.dataset.data.reduce((a, b) => a + b, 0);
                  const value = ctx.raw;
                  const percent = ((value / total) * 100).toFixed(1);
                  return `${ctx.label}: ${value} (${percent}%)`;
                }
              }
            }
          },
          layout: {
            padding: {
              top: 20,
              bottom: 30
            }
          }
        }
      });
    }
  },
  unmounted() {
    if (this.chart) {
      this.chart.destroy(); // 컴포넌트 해제 시 메모리 정리
    }
  }
};
</script>

<style scoped>
canvas {
  max-width: 100%;
}
</style>
