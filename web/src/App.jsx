import { useEffect, useMemo, useRef, useState } from 'react';
import * as echarts from 'echarts';

const SCENARIOS = [
  {
    id: 'straight',
    title: '直线路径跟随',
    file: './data/straight.json',
    tag: '基础路径任务'
  },
  {
    id: 'circle',
    title: '圆形路径跟随',
    file: './data/circle.json',
    tag: '连续转向任务'
  },
  {
    id: 's_curve',
    title: 'S 形路径跟随',
    file: './data/s_curve.json',
    tag: '连续机动任务'
  }
];

const ACCEPTANCE_RULES = [
  {
    key: 'steadyMaxAbsCrossTrackError',
    label: '稳态最大横向误差',
    unit: 'm',
    threshold: '< 1.5 m',
    pass: (value) => value < 1.5
  },
  {
    key: 'meanSpeed',
    label: '平均航速',
    unit: 'm/s',
    threshold: '1.0 ± 0.1 m/s',
    pass: (value) => Math.abs(value - 1.0) <= 0.1
  },
  {
    key: 'xSaturationRatio',
    label: '推力饱和比例',
    unit: '%',
    threshold: '< 5%',
    pass: (value) => value < 0.05,
    formatter: formatPercent
  },
  {
    key: 'nSaturationRatio',
    label: '偏航力矩饱和比例',
    unit: '%',
    threshold: '< 5%',
    pass: (value) => value < 0.05,
    formatter: formatPercent
  }
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

function radToDeg(value) {
  return (value ?? 0) * 180 / Math.PI;
}

function computeDerivedMetrics(data) {
  if (!data?.series?.length) {
    return {};
  }

  const series = data.series;
  const start = series[0];
  const end = series[series.length - 1];
  const duration = end.time - start.time;
  const steadyStartTime = start.time + duration * 0.3;
  const steadySamples = series.filter((point) => point.time >= steadyStartTime);
  const absErrors = steadySamples.map((point) => Math.abs(point.crossTrackError));
  const steadyMax = Math.max(...absErrors);
  const convergenceThreshold = 0.5;
  const convergence = findConvergenceTime(series, convergenceThreshold);
  const dt = series.length > 1 ? series[1].time - series[0].time : 0;
  const inputSmoothness = series.slice(1).reduce((sum, point, index) => {
    const prev = series[index];
    return sum + Math.abs(point.xCommand - prev.xCommand) + Math.abs(point.nCommand - prev.nCommand);
  }, 0);

  return {
    initialCrossTrackError: Math.abs(start.crossTrackError),
    steadyStartTime,
    steadyMaxAbsCrossTrackError: steadyMax,
    convergenceThreshold,
    convergenceTime: convergence,
    simulationDuration: duration,
    timeStep: dt,
    inputSmoothness
  };
}

function findConvergenceTime(series, threshold) {
  for (let index = 0; index < series.length; index += 1) {
    const remainingStable = series.slice(index).every((point) => Math.abs(point.crossTrackError) <= threshold);
    if (remainingStable) {
      return series[index].time;
    }
  }
  return null;
}

function makeEvaluationRows(summary, derived) {
  const values = { ...summary, ...derived };
  return ACCEPTANCE_RULES.map((rule) => {
    const value = values[rule.key];
    const status = rule.pass(value) ? '通过' : '警告';
    const formatter = rule.formatter ?? ((number) => `${formatNumber(number)} ${rule.unit}`);
    return {
      label: rule.label,
      value: formatter(value),
      threshold: rule.threshold,
      status
    };
  });
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

    chart.setOption({
      animation: false,
      tooltip: {
        trigger: 'item',
        valueFormatter: (value) => formatNumber(value, 3)
      },
      legend: {
        top: 4,
        data: ['参考路径', 'AUV 实际轨迹', '当前 AUV']
      },
      grid: { left: 70, right: 30, top: 62, bottom: 62 },
      xAxis: {
        type: 'value',
        name: 'x 惯性系位置 [m]',
        nameLocation: 'middle',
        nameGap: 42,
        splitLine: { lineStyle: { color: '#e1e7ef' } }
      },
      yAxis: {
        type: 'value',
        name: 'y 惯性系位置 [m]',
        nameLocation: 'middle',
        nameGap: 50,
        splitLine: { lineStyle: { color: '#e1e7ef' } },
        scale: true
      },
      series: [
        {
          name: '参考路径',
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
          name: '当前 AUV',
          type: 'scatter',
          symbol: 'triangle',
          symbolSize: 22,
          symbolRotate: -radToDeg(current.psi) + 90,
          itemStyle: { color: '#dc2626' },
          data: [[current.x, current.y]]
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

  return <div ref={chartRef} className="chart trajectory-chart" />;
}
function TimeSeriesChart({ data, sampleIndex, title, series, yAxisName, markLines = [] }) {
  const chartRef = useRef(null);

  useEffect(() => {
    if (!chartRef.current || !data) {
      return undefined;
    }

    const chart = echarts.init(chartRef.current);
    const currentTime = data.series[sampleIndex]?.time ?? 0;
    const decoratedSeries = series.map((item, index) => ({
      name: item.name,
      type: 'line',
      symbol: 'none',
      lineStyle: {
        width: item.width ?? 2,
        color: item.color,
        type: item.dashed ? 'dashed' : 'solid'
      },
      data: data.series.map((point) => [point.time, item.value(point)]),
      markLine: index === 0
        ? {
            symbol: 'none',
            label: { formatter: '{b}', color: '#991b1b' },
            data: [
              {
                name: '当前时刻',
                xAxis: currentTime,
                lineStyle: { color: '#dc2626', width: 2 }
              },
              ...markLines
            ]
          }
        : undefined
    }));

    chart.setOption({
      animation: false,
      tooltip: {
        trigger: 'axis',
        valueFormatter: (value) => formatNumber(value, 3)
      },
      legend: { top: 4 },
      grid: { left: 72, right: 28, top: 58, bottom: 64 },
      xAxis: {
        type: 'value',
        name: '仿真时间 t [s]',
        nameLocation: 'middle',
        nameGap: 42,
        splitLine: { lineStyle: { color: '#e1e7ef' } }
      },
      yAxis: {
        type: 'value',
        name: yAxisName,
        nameLocation: 'middle',
        nameGap: 52,
        splitLine: { lineStyle: { color: '#e1e7ef' } },
        scale: true
      },
      series: decoratedSeries
    });

    const resize = () => chart.resize();
    window.addEventListener('resize', resize);
    return () => {
      window.removeEventListener('resize', resize);
      chart.dispose();
    };
  }, [data, sampleIndex, series, yAxisName, markLines]);

  return (
    <div className="chart-card">
      <div className="chart-copy">
        <h3>{title}</h3>
      </div>
      <div ref={chartRef} className="small-chart" />
    </div>
  );
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
    ['x', `${formatNumber(sample?.x)} m`],
    ['y', `${formatNumber(sample?.y)} m`],
    ['ψ', `${formatNumber(radToDeg(sample?.psi))}°`],
    ['u', `${formatNumber(sample?.u)} m/s`],
    ['r', `${formatNumber(radToDeg(sample?.r))} °/s`]
  ];

  return (
    <section className="side-card">
      <div className="panel-title-row dense-title">
        <h3>当前状态</h3>
        <span>{formatNumber(sample?.time, 2)} s</span>
      </div>
      <div className="state-panel">
        {rows.map(([label, value]) => (
          <div key={label}>
            <span>{label}</span>
            <strong>{value}</strong>
          </div>
        ))}
      </div>
    </section>
  );
}

function ScenarioCharts({ data, sampleIndex, derived }) {
  const limits = data.limits ?? {};
  const convergenceThreshold = derived.convergenceThreshold ?? 0.5;

  return (
    <div className="chart-grid">
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="横向路径误差 ey"
        yAxisName="横向路径误差 ey [m]"
        markLines={[
          { name: '+收敛阈值', yAxis: convergenceThreshold, lineStyle: { color: '#f97316', type: 'dashed' } },
          { name: '-收敛阈值', yAxis: -convergenceThreshold, lineStyle: { color: '#f97316', type: 'dashed' } }
        ]}
        series={[
          { name: '横向路径误差 ey', color: '#2563eb', value: (point) => point.crossTrackError },
          { name: '零误差线', color: '#94a3b8', dashed: true, value: () => 0 }
        ]}
      />
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="航向误差 eψ"
        yAxisName="航向误差 eψ [°]"
        series={[
          { name: '航向误差 eψ', color: '#9333ea', value: (point) => radToDeg(point.headingError) },
          { name: '零误差线', color: '#94a3b8', dashed: true, value: () => 0 }
        ]}
      />
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="纵向速度 u"
        yAxisName="纵向速度 u [m/s]"
        markLines={[
          { name: '目标航速', yAxis: 1.0, lineStyle: { color: '#16a34a', type: 'dashed' } }
        ]}
        series={[
          { name: '纵向速度 u', color: '#16a34a', value: (point) => point.u }
        ]}
      />
      <TimeSeriesChart
        data={data}
        sampleIndex={sampleIndex}
        title="控制输入曲线"
        yAxisName="控制输入 [N / N·m]"
        markLines={[
          { name: 'Xmax', yAxis: limits.xCommandMax ?? 80, lineStyle: { color: '#dc2626', type: 'dashed' } },
          { name: 'Xmin', yAxis: limits.xCommandMin ?? -80, lineStyle: { color: '#dc2626', type: 'dashed' } },
          { name: 'Nmax', yAxis: limits.nCommandMax ?? 20, lineStyle: { color: '#7c3aed', type: 'dashed' } },
          { name: 'Nmin', yAxis: limits.nCommandMin ?? -20, lineStyle: { color: '#7c3aed', type: 'dashed' } }
        ]}
        series={[
          { name: 'Xcmd 纵向推力', color: '#0f766e', value: (point) => point.xCommand },
          { name: 'Ncmd', color: '#7c3aed', value: (point) => point.nCommand }
        ]}
      />
    </div>
  );
}

function AcceptancePanel({ rows }) {
  return (
    <div className="acceptance-panel">
      <div className="panel-title-row">
        <h3>验收指标判断</h3>
      </div>
      <div className="acceptance-table">
        <div className="table-head">指标</div>
        <div className="table-head">当前值</div>
        <div className="table-head">阈值</div>
        <div className="table-head">状态</div>
        {rows.map((row) => (
          <div className="table-row-group" key={row.label}>
            <div className="table-row">{row.label}</div>
            <div className="table-row numeric">{row.value}</div>
            <div className="table-row">{row.threshold}</div>
            <div className="table-row">
              <span className={row.status === '通过' ? 'badge pass' : 'badge warn'}>{row.status}</span>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

function getOverallStatus(rows) {
  return rows.every((row) => row.status === '通过') ? '通过' : '警告';
}

function KeyResultPanel({ summary, evaluationRows }) {
  const overallStatus = getOverallStatus(evaluationRows);
  const items = [
    ['控制输入峰值', `${formatNumber(summary.maxAbsXCommand)} N / ${formatNumber(summary.maxAbsNCommand)} N·m`],
    ['饱和比例', `X ${formatPercent(summary.xSaturationRatio)} / N ${formatPercent(summary.nSaturationRatio)}`]
  ];

  return (
    <section className="side-card">
      <div className="panel-title-row dense-title">
        <h3>核心指标</h3>
        <span className={overallStatus === '通过' ? 'badge pass' : 'badge warn'}>{overallStatus}</span>
      </div>
      <div className="key-metric-list">
        {items.map(([label, value]) => (
          <div key={label}>
            <span>{label}</span>
            <strong>{value}</strong>
          </div>
        ))}
      </div>
    </section>
  );
}

function TopBar({ data, selectedScenario }) {
  const scenarioName = data?.scenario?.name ?? selectedScenario?.title ?? '读取中';
  const controller = data?.scenario?.controller ?? 'LOS + PID';
  const status = data ? '正常' : '读取中';
  const points = data?.series?.length ?? 0;

  return (
    <header className="top-bar">
      <div className="project-title">
        <span>AUV 单体控制算法模块及可视化</span>
        <strong>AUV 单体控制算法仿真可视化系统</strong>
      </div>
      <div className="top-meta">
        <div><span>当前场景</span><strong>{scenarioName}</strong></div>
        <div><span>控制器</span><strong>{controller}</strong></div>
        <div><span>运行状态</span><strong>{status}</strong></div>
        <div><span>数据点数</span><strong>{points}</strong></div>
      </div>
    </header>
  );
}

function ScenarioSelector({ selectedId, onSelect, scenarioSummaries }) {
  return (
    <div className="scenario-grid">
      {SCENARIOS.map((scenario) => {
        const summary = scenarioSummaries[scenario.id];
        return (
          <button
            className={scenario.id === selectedId ? 'scenario-card active' : 'scenario-card'}
            key={scenario.id}
            type="button"
            onClick={() => onSelect(scenario.id)}
          >
            <span>{scenario.tag}</span>
            <strong>{scenario.title}</strong>
            {summary && (
              <small>
                推力饱和 {formatPercent(summary.xSaturationRatio)}
              </small>
            )}
          </button>
        );
      })}
    </div>
  );
}

export default function App() {
  const [selectedId, setSelectedId] = useState('straight');
  const [scenarioData, setScenarioData] = useState(null);
  const [scenarioSummaries, setScenarioSummaries] = useState({});
  const [loadState, setLoadState] = useState('loading');
  const [sampleIndex, setSampleIndex] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);

  const selectedScenario = useMemo(
    () => SCENARIOS.find((scenario) => scenario.id === selectedId),
    [selectedId]
  );

  useEffect(() => {
    Promise.all(
      SCENARIOS.map((scenario) => fetch(scenario.file).then((response) => response.json()).then((data) => [scenario.id, data.summary]))
    )
      .then((entries) => setScenarioSummaries(Object.fromEntries(entries)))
      .catch(() => setScenarioSummaries({}));
  }, []);

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
  const derived = useMemo(() => computeDerivedMetrics(scenarioData), [scenarioData]);
  const evaluationRows = useMemo(
    () => (scenarioData ? makeEvaluationRows(scenarioData.summary, derived) : []),
    [scenarioData, derived]
  );

  return (
    <main className="app-shell">
      <TopBar data={scenarioData} selectedScenario={selectedScenario} />

      <section className="panel compact-panel">
        <div className="panel-title-row">
          <h2>场景切换</h2>
        </div>
        <ScenarioSelector selectedId={selectedId} onSelect={setSelectedId} scenarioSummaries={scenarioSummaries} />
      </section>

      <section className="panel">
        {loadState === 'loading' && <p className="notice">正在读取场景数据...</p>}
        {loadState === 'error' && <p className="notice error">场景数据读取失败，请检查 web/public/data 目录。</p>}
        {loadState === 'ready' && scenarioData && (
          <>
            <div className="dashboard-grid">
              <section className="main-view">
                <div className="chart-card main-trajectory">
                  <div className="chart-copy">
                    <h3>轨迹图 / 仿真回放主视图</h3>
                  </div>
                  <TrajectoryChart data={scenarioData} sampleIndex={sampleIndex} />
                </div>
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
              </section>

              <aside className="dashboard-side">
                <StatePanel sample={currentSample} />
                <KeyResultPanel summary={scenarioData.summary} evaluationRows={evaluationRows} />
                <AcceptancePanel rows={evaluationRows} />
              </aside>
            </div>

            <ScenarioCharts data={scenarioData} sampleIndex={sampleIndex} derived={derived} />
          </>
        )}
      </section>
    </main>
  );
}
