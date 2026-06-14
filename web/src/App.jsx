import { useEffect, useMemo, useRef, useState } from 'react';
import * as echarts from 'echarts';

const SCENARIOS = [
  {
    id: 'straight',
    title: '直线路径跟随能力',
    file: './data/straight.json',
    tag: '基础路径任务'
  },
  {
    id: 'circle',
    title: '圆形路径跟随能力',
    file: './data/circle.json',
    tag: '连续转向任务'
  },
  {
    id: 's_curve',
    title: 'S 形路径跟随能力',
    file: './data/s_curve.json',
    tag: '连续机动任务'
  }
];

const MODULES = [
  ['3-DOF 仿真模型', '水平面欠驱动 AUV 运动仿真'],
  ['LOS 路径制导', '根据参考路径生成期望航向'],
  ['航向 PID 控制', '输出偏航力矩控制航向误差'],
  ['速度控制', '输出纵向推力维持航速'],
  ['执行器限幅', '约束推力和偏航力矩命令'],
  ['可视化展示', '展示轨迹、状态和控制结果']
];

function formatNumber(value, digits = 3) {
  if (typeof value !== 'number' || Number.isNaN(value)) {
    return 'N/A';
  }
  return value.toFixed(digits);
}

function formatPercent(value) {
  if (typeof value !== 'number' || Number.isNaN(value)) {
    return 'N/A';
  }
  return `${(value * 100).toFixed(2)}%`;
}

function TrajectoryChart({ data, sampleIndex }) {
  const chartRef = useRef(null);

  useEffect(() => {
    if (!chartRef.current || !data) {
      return undefined;
    }

    const chart = echarts.init(chartRef.current);
    const actual = data.series.map((point) => [point.x, point.y]);
    const reference = data.series
      .filter((point) => Number.isFinite(point.xRef) && Number.isFinite(point.yRef))
      .map((point) => [point.xRef, point.yRef]);
    const current = data.series[sampleIndex] ?? data.series[0];
    const arrowLength = estimateArrowLength(data.series);
    const arrowEnd = [
      current.x + arrowLength * Math.cos(current.psi),
      current.y + arrowLength * Math.sin(current.psi)
    ];

    chart.setOption({
      animation: false,
      tooltip: {
        trigger: 'axis',
        valueFormatter: (value) => formatNumber(value, 3)
      },
      legend: {
        top: 4,
        data: ['参考路径/最近参考点', 'AUV 实际轨迹']
      },
      grid: {
        left: 70,
        right: 28,
        top: 56,
        bottom: 64
      },
      xAxis: {
        type: 'value',
        name: 'x 惯性系位置 [m]',
        nameLocation: 'middle',
        nameGap: 42,
        splitLine: { lineStyle: { color: '#e5e8ef' } }
      },
      yAxis: {
        type: 'value',
        name: 'y 惯性系位置 [m]',
        nameLocation: 'middle',
        nameGap: 48,
        splitLine: { lineStyle: { color: '#e5e8ef' } },
        scale: true
      },
      series: [
        {
          name: '参考路径/最近参考点',
          type: 'line',
          symbol: 'none',
          lineStyle: { width: 2, type: 'dashed', color: '#64748b' },
          data: reference
        },
        {
          name: 'AUV 实际轨迹',
          type: 'line',
          symbol: 'none',
          lineStyle: { width: 3, color: '#0f766e' },
          data: actual
        },
        {
          name: '当前 AUV 位置',
          type: 'scatter',
          symbolSize: 14,
          itemStyle: { color: '#dc2626' },
          data: [[current.x, current.y]]
        },
        {
          name: '当前航向',
          type: 'lines',
          coordinateSystem: 'cartesian2d',
          symbol: ['none', 'arrow'],
          symbolSize: 12,
          lineStyle: { width: 3, color: '#dc2626' },
          data: [
            {
              coords: [
                [current.x, current.y],
                arrowEnd
              ]
            }
          ]
        }
      ]
    });

    const resize = () => chart.resize();
    window.addEventListener('resize', resize);
    return () => {
      window.removeEventListener('resize', resize);
      chart.dispose();
    };
  }, [data, sampleIndex]);

  return <div ref={chartRef} className="chart" />;
}

function TimeSeriesChart({ data, sampleIndex, title, description, series, yAxisName }) {
  const chartRef = useRef(null);

  useEffect(() => {
    if (!chartRef.current || !data) {
      return undefined;
    }

    const chart = echarts.init(chartRef.current);
    const currentTime = data.series[sampleIndex]?.time ?? 0;

    chart.setOption({
      animation: false,
      tooltip: {
        trigger: 'axis',
        valueFormatter: (value) => formatNumber(value, 3)
      },
      legend: { top: 4 },
      grid: { left: 70, right: 28, top: 58, bottom: 64 },
      xAxis: {
        type: 'value',
        name: 'time 仿真时间 [s]',
        nameLocation: 'middle',
        nameGap: 42,
        splitLine: { lineStyle: { color: '#e5e8ef' } }
      },
      yAxis: {
        type: 'value',
        name: yAxisName,
        nameLocation: 'middle',
        nameGap: 48,
        splitLine: { lineStyle: { color: '#e5e8ef' } },
        scale: true
      },
      series: series.map((item) => ({
        name: item.name,
        type: 'line',
        symbol: 'none',
        lineStyle: {
          width: item.width ?? 2,
          color: item.color,
          type: item.dashed ? 'dashed' : 'solid'
        },
        data: data.series.map((point) => [point.time, item.value(point)])
      })),
      graphic: [
        {
          type: 'line',
          shape: { x1: 0, y1: 0, x2: 0, y2: 1 },
          invisible: true
        }
      ]
    });

    chart.setOption({
      series: [
        ...series.map((item) => ({
          name: item.name,
          markLine: item.markCurrent
            ? {
                symbol: 'none',
                lineStyle: { color: '#dc2626', width: 2 },
                label: { formatter: '当前时刻' },
                data: [{ xAxis: currentTime }]
              }
            : undefined
        }))
      ]
    });

    const resize = () => chart.resize();
    window.addEventListener('resize', resize);
    return () => {
      window.removeEventListener('resize', resize);
      chart.dispose();
    };
  }, [data, sampleIndex, series, yAxisName]);

  return (
    <div className="chart-card">
      <div className="chart-copy">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
      <div ref={chartRef} className="small-chart" />
    </div>
  );
}

function estimateArrowLength(series) {
  if (!series?.length) {
    return 1;
  }
  const xs = series.map((point) => point.x);
  const ys = series.map((point) => point.y);
  const xSpan = Math.max(...xs) - Math.min(...xs);
  const ySpan = Math.max(...ys) - Math.min(...ys);
  return Math.max(1, Math.max(xSpan, ySpan) * 0.04);
}

function ReplayControls({
  isPlaying,
  playbackSpeed,
  sampleIndex,
  totalSamples,
  currentSample,
  onPlayPause,
  onReset,
  onSpeedChange,
  onIndexChange
}) {
  const maxIndex = Math.max(0, totalSamples - 1);

  return (
    <div className="replay-panel">
      <div className="replay-actions">
        <button type="button" className="primary-button" onClick={onPlayPause}>
          {isPlaying ? '暂停' : '播放'}
        </button>
        <button type="button" className="secondary-button" onClick={onReset}>
          重置
        </button>
        <label>
          倍速
          <select value={playbackSpeed} onChange={(event) => onSpeedChange(Number(event.target.value))}>
            <option value={0.5}>0.5x</option>
            <option value={1}>1x</option>
            <option value={2}>2x</option>
            <option value={4}>4x</option>
          </select>
        </label>
      </div>
      <div className="timeline-row">
        <span>{formatNumber(currentSample?.time ?? 0, 2)} s</span>
        <input
          type="range"
          min="0"
          max={maxIndex}
          value={sampleIndex}
          onChange={(event) => onIndexChange(Number(event.target.value))}
        />
        <span>{sampleIndex + 1} / {totalSamples}</span>
      </div>
    </div>
  );
}

function StatePanel({ sample }) {
  const rows = [
    ['当前时间', `${formatNumber(sample?.time, 2)} s`],
    ['x 位置', `${formatNumber(sample?.x)} m`],
    ['y 位置', `${formatNumber(sample?.y)} m`],
    ['航向角', `${formatNumber((sample?.psi ?? 0) * 180 / Math.PI)} deg`],
    ['纵向速度 u', `${formatNumber(sample?.u)} m/s`],
    ['横向误差', `${formatNumber(sample?.crossTrackError)} m`],
    ['航向误差', `${formatNumber((sample?.headingError ?? 0) * 180 / Math.PI)} deg`],
    ['纵向推力 X', `${formatNumber(sample?.xCommand)} N`],
    ['偏航力矩 N', `${formatNumber(sample?.nCommand)} N*m`]
  ];

  return (
    <div className="state-panel">
      {rows.map(([label, value]) => (
        <div key={label}>
          <span>{label}</span>
          <strong>{value}</strong>
        </div>
      ))}
    </div>
  );
}

function EngineeringNotes() {
  return (
    <section className="panel">
      <h2>交付展示说明</h2>
      <div className="note-grid">
        <div>
          <strong>模块已完成</strong>
          <p>当前版本已经形成 AUV 单体控制算法链路，可完成路径制导、控制输入生成、仿真输出和可视化展示。</p>
        </div>
        <div>
          <strong>展示方式</strong>
          <p>APP 读取本地 JSON 数据，支持离线打包，适合在断网环境下进行甲方现场演示。</p>
        </div>
        <div>
          <strong>当前阶段</strong>
          <p>阶段四重点完善工程说明、状态指标和曲线图，为后续桌面封装和更丰富动画打基础。</p>
        </div>
      </div>
    </section>
  );
}

function ScenarioCharts({ data, sampleIndex }) {
  const limits = data.limits ?? {};
  const radToDeg = (value) => (value ?? 0) * 180 / Math.PI;

  return (
    <div className="chart-grid">
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="横向路径误差"
        description="该图展示 AUV 相对参考路径的横向偏差，用于观察路径跟随是否逐渐稳定。"
        yAxisName="cross-track error [m]"
        series={[
          { name: '横向误差', color: '#2563eb', markCurrent: true, value: (point) => point.crossTrackError },
          { name: '零误差参考', color: '#94a3b8', dashed: true, value: () => 0 }
        ]}
      />
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="航向误差"
        description="该图展示实际航向与 LOS 期望航向之间的误差，用于判断航向控制是否平稳。"
        yAxisName="heading error [deg]"
        series={[
          { name: '航向误差', color: '#9333ea', markCurrent: true, value: (point) => radToDeg(point.headingError) },
          { name: '零误差参考', color: '#94a3b8', dashed: true, value: () => 0 }
        ]}
      />
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="纵向速度"
        description="该图展示 AUV 体坐标系纵向速度，用于检查速度控制是否保持稳定。"
        yAxisName="surge speed u [m/s]"
        series={[
          { name: '纵向速度 u', color: '#16a34a', markCurrent: true, value: (point) => point.u }
        ]}
      />
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="纵向推力命令"
        description="该图展示纵向推力命令和限幅线，用于判断推进输入是否长期饱和。"
        yAxisName="X command [N]"
        series={[
          { name: 'X_cmd', color: '#0f766e', markCurrent: true, value: (point) => point.xCommand },
          { name: 'X 上限', color: '#dc2626', dashed: true, value: () => limits.xCommandMax ?? 80 },
          { name: 'X 下限', color: '#dc2626', dashed: true, value: () => limits.xCommandMin ?? -80 }
        ]}
      />
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="偏航力矩命令"
        description="该图展示偏航力矩命令和限幅线，用于判断转向控制输入是否平滑、是否触碰饱和。"
        yAxisName="N command [N*m]"
        series={[
          { name: 'N_cmd', color: '#7c3aed', markCurrent: true, value: (point) => point.nCommand },
          { name: 'N 上限', color: '#dc2626', dashed: true, value: () => limits.nCommandMax ?? 20 },
          { name: 'N 下限', color: '#dc2626', dashed: true, value: () => limits.nCommandMin ?? -20 }
        ]}
      />
    </div>
  );
}

function SummaryCards({ summary }) {
  const cards = [
    ['RMS 横向误差', `${formatNumber(summary.rmsCrossTrackError)} m`],
    ['最大横向误差', `${formatNumber(summary.maxAbsCrossTrackError)} m`],
    ['平均航速', `${formatNumber(summary.meanSpeed)} m/s`],
    ['最大推力命令', `${formatNumber(summary.maxAbsXCommand)} N`],
    ['最大偏航力矩', `${formatNumber(summary.maxAbsNCommand)} N*m`],
    ['推力饱和比例', formatPercent(summary.xSaturationRatio)]
  ];

  return (
    <div className="metric-grid">
      {cards.map(([label, value]) => (
        <div className="metric-card" key={label}>
          <span>{label}</span>
          <strong>{value}</strong>
        </div>
      ))}
    </div>
  );
}

function ScenarioSelector({ selectedId, onSelect }) {
  return (
    <div className="scenario-grid">
      {SCENARIOS.map((scenario) => (
        <button
          className={scenario.id === selectedId ? 'scenario-card active' : 'scenario-card'}
          key={scenario.id}
          type="button"
          onClick={() => onSelect(scenario.id)}
        >
          <span>{scenario.tag}</span>
          <strong>{scenario.title}</strong>
        </button>
      ))}
    </div>
  );
}

function FlowDiagram() {
  const nodes = ['参考路径', 'LOS 制导', '航向/速度控制', '执行器限幅', '3-DOF 模型', '状态输出', '可视化展示'];
  return (
    <div className="flow-row">
      {nodes.map((node, index) => (
        <div className="flow-item" key={node}>
          <span>{node}</span>
          {index < nodes.length - 1 && <b>→</b>}
        </div>
      ))}
    </div>
  );
}

export default function App() {
  const [selectedId, setSelectedId] = useState('straight');
  const [scenarioData, setScenarioData] = useState(null);
  const [loadState, setLoadState] = useState('loading');
  const [sampleIndex, setSampleIndex] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);

  const selectedScenario = useMemo(
    () => SCENARIOS.find((scenario) => scenario.id === selectedId),
    [selectedId]
  );

  useEffect(() => {
    let cancelled = false;
    setLoadState('loading');
    setSampleIndex(0);
    setIsPlaying(false);

    fetch(selectedScenario.file)
      .then((response) => {
        if (!response.ok) {
          throw new Error(`读取数据失败: ${response.status}`);
        }
        return response.json();
      })
      .then((data) => {
        if (!cancelled) {
          setScenarioData(data);
          setLoadState('ready');
        }
      })
      .catch(() => {
        if (!cancelled) {
          setScenarioData(null);
          setLoadState('error');
        }
      });

    return () => {
      cancelled = true;
    };
  }, [selectedScenario]);

  useEffect(() => {
    if (!isPlaying || !scenarioData) {
      return undefined;
    }

    const interval = window.setInterval(() => {
      setSampleIndex((current) => {
        const next = current + Math.max(1, Math.round(5 * playbackSpeed));
        if (next >= scenarioData.series.length - 1) {
          setIsPlaying(false);
          return scenarioData.series.length - 1;
        }
        return next;
      });
    }, 50);

    return () => window.clearInterval(interval);
  }, [isPlaying, playbackSpeed, scenarioData]);

  const currentSample = scenarioData?.series?.[sampleIndex] ?? null;

  return (
    <main className="app-shell">
      <section className="hero">
        <div>
          <p className="eyebrow">AUV 单体控制算法模块及可视化</p>
          <h1>工程交付展示 APP</h1>
          <p className="summary">
            本 APP 用于展示 AUV 单体控制算法模块已具备的路径制导、航向控制、
            速度控制、执行器限幅、仿真输出和可视化能力。
          </p>
        </div>
        <div className="hero-status">
          <span>阶段二</span>
          <strong>基础 APP 页面与数据读取</strong>
        </div>
      </section>

      <section className="panel">
        <h2>已完成模块</h2>
        <div className="module-grid">
          {MODULES.map(([title, text]) => (
            <div className="module-card" key={title}>
              <strong>{title}</strong>
              <span>{text}</span>
            </div>
          ))}
        </div>
      </section>

      <section className="panel">
        <h2>控制算法流程</h2>
        <FlowDiagram />
      </section>

      <EngineeringNotes />

      <section className="panel">
        <h2>演示场景选择</h2>
        <ScenarioSelector selectedId={selectedId} onSelect={setSelectedId} />
      </section>

      <section className="panel">
        {loadState === 'loading' && <p className="notice">正在读取场景数据...</p>}
        {loadState === 'error' && <p className="notice error">场景数据读取失败，请检查 web/public/data 目录。</p>}
        {loadState === 'ready' && scenarioData && (
          <>
            <div className="scenario-header">
              <div>
                <p className="eyebrow">当前演示</p>
                <h2>{scenarioData.scenario.name}</h2>
                <p className="summary">{scenarioData.scenario.description}</p>
              </div>
              <div className="status-stack">
                <div className="status-pill">运行状态：{scenarioData.summary.status === 'normal' ? '正常' : scenarioData.summary.status}</div>
                <div className="status-pill neutral">数据点：{scenarioData.series.length}</div>
                <div className="status-pill neutral">控制器：{scenarioData.scenario.controller}</div>
              </div>
            </div>
            <SummaryCards summary={scenarioData.summary} />
            <ReplayControls
              isPlaying={isPlaying}
              playbackSpeed={playbackSpeed}
              sampleIndex={sampleIndex}
              totalSamples={scenarioData.series.length}
              currentSample={currentSample}
              onPlayPause={() => setIsPlaying((value) => !value)}
              onReset={() => {
                setSampleIndex(0);
                setIsPlaying(false);
              }}
              onSpeedChange={setPlaybackSpeed}
              onIndexChange={(value) => {
                setSampleIndex(value);
                setIsPlaying(false);
              }}
            />
            <StatePanel sample={currentSample} />
            <div className="chart-card">
              <div className="chart-copy">
                <h3>基础轨迹图</h3>
                <p>该图展示 AUV 实际轨迹和参考路径/最近参考点，蓝色标记表示当前回放时刻的 AUV 位置，短箭头表示当前航向。</p>
              </div>
              <TrajectoryChart data={scenarioData} sampleIndex={sampleIndex} />
            </div>
            <ScenarioCharts data={scenarioData} sampleIndex={sampleIndex} />
            <div className="conclusion">
              <strong>演示结论</strong>
              <p>{scenarioData.scenario.conclusion}</p>
            </div>
          </>
        )}
      </section>
    </main>
  );
}
