<script>
import { Chart, ScatterController, PointElement, LinearScale, Tooltip, Legend } from 'chart.js'

Chart.register(ScatterController, PointElement, LinearScale, Tooltip, Legend)

export default {
  props: ['points'],
  mounted() {
    const ctx = this.$refs.canvas.getContext('2d')
    this.chart = new Chart(ctx, {
      type: 'scatter',
      data: {
        datasets: [
          {
            label: '위치 이동',
            data: this.points.map((p) => ({ x: p.x, y: p.y })),
            backgroundColor: this.points.map(
              (p) => `rgba(102, 242, 197, ${Math.min(p.z / 150, 1)})`,
            ),
            pointRadius: 6,
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        layout: {
          padding: {
            bottom: 20, // ✅ 아래 여백 확보
          },
        },
        scales: {
          x: {
            min: -300,
            max: 300,
            title: {
              display: true,
              text: 'X 좌표',
            },
            grid: {
              color: 'rgba(255,255,255,0.1)',
            },
          },
          y: {
            min: -100,
            max: 320,
            title: {
              display: true,
              text: 'Y 좌표',
            },
            grid: {
              color: 'rgba(255,255,255,0.1)',
            },
          },
        },
        plugins: {
          legend: { display: false },
        },
      },
    })
  },
  watch: {
    points(newPoints) {
      this.chart.data.datasets[0].data = newPoints.map((p) => ({ x: p.x, y: p.y }))
      this.chart.data.datasets[0].backgroundColor = newPoints.map(
        (p) => `rgba(102, 242, 197, ${Math.min(p.z / 150, 1)})`,
      )
      this.chart.update()
    },
  },
}
</script>

<template>
  <canvas ref="canvas"></canvas>
</template>
