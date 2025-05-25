<template>
  <canvas ref="canvas" style="width: 100%; height: 100%"></canvas>
</template>

<script>
import { Chart, DoughnutController, ArcElement, Tooltip, Legend } from 'chart.js';
Chart.register(DoughnutController, ArcElement, Tooltip, Legend);

export default {
  props: ['data'],
  mounted() {
    const ctx = this.$refs.canvas.getContext('2d');
    new Chart(ctx, {
      type: 'doughnut',
      data: {
        labels: ['음성', '제스처', '버튼'],
        datasets: [{
          data: this.data,
          backgroundColor: ['#66f2c5', '#42a5f5', '#ffca28'],
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
}
</script>
