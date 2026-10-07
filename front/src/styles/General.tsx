import styled from 'styled-components'

/* Dark, low-contrast desktop palette shared by all panels. */
export const palette = {
  appBackground: '#141619',
  surface: '#1b1d21',
  surfaceRaised: '#212429',
  surfaceHover: '#2a2e34',
  console: '#111316',
  border: '#2a2d33',
  borderStrong: '#3a3e46',
  text: '#e2e4e8',
  textSecondary: '#b2b7bf',
  textMuted: '#8f959e',
  textDim: '#626872',
  green: '#4fbf8b',
  orange: '#e5954a',
  /* Primary accent: primary actions, active selection and focus. */
  accent: '#cf7f35',
  accentHover: '#db8b42',
  /* Neutral icon buttons (folder, terminal, info). */
  icon: '#9aa0a9',
  iconHover: '#e2e4e8',
  /* Reserved for the pulse processing path in the pipeline logs. */
  blue: '#4a8ad4',
  red: '#cf5149',
  redHover: '#db5d55',
  yellow: '#d8b445',
  monoFont: "'JetBrains Mono', 'SF Mono', Menlo, Consolas, 'DejaVu Sans Mono', monospace",
}

export const selectChevron =
  'url("data:image/svg+xml,%3Csvg xmlns=\'http://www.w3.org/2000/svg\' ' +
  'width=\'10\' height=\'6\' viewBox=\'0 0 10 6\'%3E' +
  '%3Cpath d=\'M1 1l4 4 4-4\' fill=\'none\' stroke=\'%238f959e\' stroke-width=\'1.4\'/%3E%3C/svg%3E")'

export const Select = styled.select`
  width: 190px;
  height: 24px;
  box-sizing: border-box;
  padding: 0 22px 0 8px;
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  outline: none;
  transition: border-color 0.15s, background-color 0.15s;
  appearance: none;
  margin-right: 17px;
  background: ${palette.surfaceRaised} ${selectChevron} no-repeat right 8px center;
  color: ${palette.text};
  font-size: 12px;
  font-family: inherit;

  &:hover:not(:disabled) {
    border-color: #4a4f58;
  }

  &:focus {
    border-color: ${palette.accent};
  }

  /* Set explicitly (Chromium also fades disabled selects by default) so that other disabled
     controls can match it. */
  &:disabled {
    color: ${palette.textMuted};
    background-color: ${palette.surface};
    border-color: ${palette.border};
    opacity: 0.7;
  }
`

// Shared width for configuration panels
export const CONFIG_PANEL_WIDTH = 350

/* The pipeline logs toolbar starts at this offset; the log body below it has a fixed height that
   does not depend on the window or the log content. Change the offset to move the logs; the
   telemetry panels sit directly above them. */
export const PIPELINE_LOG_OFFSET_FROM_TOP = 538
export const PIPELINE_LOG_BODY_HEIGHT = 550

/* Telemetry panels (header strip + body), stacked upwards from the top of the pipeline logs. */
export const DASHBOARD_HEADER_HEIGHT = 28
export const DASHBOARD_PANEL_HEIGHT = 350
export const DASHBOARD_PANEL_OFFSET_FROM_TOP =
  PIPELINE_LOG_OFFSET_FROM_TOP - DASHBOARD_PANEL_HEIGHT - DASHBOARD_HEADER_HEIGHT

/* Pipeline content area height (px). */
export const PIPELINE_WRAPPER_HEIGHT = 1030

export const StyledPanel = styled.div<{ isGrayedOut?: boolean }>`
  padding: 10px 0px 12px 16px;
  background-color: transparent;
  color: ${palette.text};
  ${({ isGrayedOut }) =>
    isGrayedOut &&
    `
    filter: grayscale(100%);
    opacity: 0.4;
    transition: filter 0.3s ease, opacity 0.3s ease;
    pointer-events: none;
  `}
`

export const ConfigPanel = styled(StyledPanel)`
  padding: 10px 16px 12px 20px;
  border-top: 1px solid ${palette.border};
`

/* Width of the right-hand region (status strip, telemetry block, pipeline logs). */
export const PIPELINE_LOG_WIDTH = 983

/* Telemetry columns, from the right edge of the window: Stimulation, Statistics, Experiment.
   Together they span PIPELINE_LOG_WIDTH. */
export const TELEMETRY_STIMULATION_WIDTH = 328
export const TELEMETRY_STATISTICS_WIDTH = 321
export const TELEMETRY_STATISTICS_RIGHT = TELEMETRY_STIMULATION_WIDTH
export const TELEMETRY_EXPERIMENT_RIGHT = TELEMETRY_STATISTICS_RIGHT + TELEMETRY_STATISTICS_WIDTH
export const TELEMETRY_EXPERIMENT_WIDTH = PIPELINE_LOG_WIDTH - TELEMETRY_EXPERIMENT_RIGHT

/* Header strip and body of the fixed telemetry panels (Experiment, Statistics, Stimulation).
   Adjacent panels share their left border as a divider, so together they read as one region. */
export const TelemetryTitle = styled.div`
  position: fixed;
  top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP}px;
  height: ${DASHBOARD_HEADER_HEIGHT}px;
  box-sizing: border-box;
  padding: 0 12px;
  z-index: 1001;
  display: flex;
  align-items: center;
  background-color: ${palette.surface};
  border-top: 1px solid ${palette.border};
  border-left: 1px solid ${palette.border};
  color: ${palette.textSecondary};
  font-size: 11px;
  font-weight: 600;
  letter-spacing: 0.06em;
  text-transform: uppercase;
`

export const TelemetryPanel = styled.div`
  position: fixed;
  top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP + DASHBOARD_HEADER_HEIGHT}px;
  height: ${DASHBOARD_PANEL_HEIGHT}px;
  box-sizing: border-box;
  padding: 8px 12px;
  z-index: 1000;
  background-color: ${palette.surface};
  border-left: 1px solid ${palette.border};
  color: ${palette.text};
`

export interface ActiveProps {
  isActive?: boolean
  isHidden?: boolean
}

export const StyledButton = styled.button<ActiveProps>`
  width: 110px;
  height: 26px;
  box-sizing: border-box;

  font-size: 12px;
  font-weight: 600;
  font-family: inherit;
  padding: 0 10px;
  margin-bottom: 0;
  border: 1px solid transparent;
  border-radius: 3px;
  background-color: ${palette.accent};
  color: #ffffff;
  cursor: pointer;

  &:hover {
    background-color: ${palette.accentHover};
  }
  &:disabled {
    background-color: ${palette.surfaceRaised};
    border-color: ${palette.border};
    color: ${palette.textDim};
    cursor: default;
  }
  &:hover:disabled {
    background-color: ${palette.surfaceRaised};
  }
  transition: opacity 0.2s, background-color 0.15s;

  ${(props) =>
    props.isHidden &&
    `
    opacity: 0.0;
  `}
`

export const StyledRedButton = styled(StyledButton)`
  background-color: ${palette.red};
  &:hover {
    background-color: ${palette.redHover};
  }
`

export const TabBar = styled.div`
  margin: 0.31rem;

  a {
    text-decoration: none;
    color: ${palette.textMuted};
    padding: 0.31rem;
    display: inline-block;
    transition: color 0.3s ease;

    &:hover {
      color: ${palette.textSecondary};
    }

    &.active {
      color: ${palette.text};
      font-weight: 600;
    }
  }
`

export const ProjectRow = styled.div`
  display: flex;
  justify-content: flex-start;
  align-items: center;
  gap: 6px;
  margin-bottom: 12px;
`

/* General config-related */
export const ConfigRow = styled.div`
  display: flex;
  justify-content: flex-start;
  align-items: center;
  gap: 5px;
  margin-bottom: 4px;
  padding-right: 0px;
  min-height: 20px;
`

export const CloseConfigRow = styled(ConfigRow)`
  margin-bottom: 4px;
`

export const ConfigLabel = styled.label`
  width: 185px;
  font-size: 12px;
  color: ${palette.textSecondary};
  display: inline-flex;
  justify-content: flex-start;
  align-items: center;
`

export const IndentedLabel = styled(ConfigLabel)`
  padding-left: 10px;
`

export const ConfigValue = styled.div`
  display: inline-flex;
  justify-content: flex-end;
  align-items: center;
  width: 190px;
  text-align: right;
  font-size: 12px;
  font-variant-numeric: tabular-nums;
  color: ${palette.text};
  margin-right: 17px;
`

export const NotesValue = styled.div`
  width: 190px;
  flex-shrink: 0;
  text-align: right;
  font-size: 12px;
  line-height: 16px;
  color: ${palette.text};
  margin-right: 17px;
  white-space: pre-wrap;
  word-wrap: break-word;
  overflow-wrap: break-word;
  max-height: 48px;
  overflow-y: auto;
`

export const SmallerTitle = styled.h2`
  font-size: 12px;
  text-align: center;
  margin-bottom: 18px;
  margin-right: 18px;
  font-weight: 600;
  color: ${palette.text};
`

export const ConfigTitle = styled(SmallerTitle)`
  margin: 0 0 10px 0;
  text-align: left;
  font-size: 11px;
  letter-spacing: 0.06em;
  text-transform: uppercase;
  color: ${palette.textSecondary};
`

/* A section header line that doubles as a status line, so transient status needs no space of its
   own and the section keeps the same height in every state. */
export const ConfigTitleRow = styled.div`
  display: flex;
  align-items: baseline;
  justify-content: space-between;
  gap: 8px;
  margin-bottom: 10px;
  padding-right: 17px;
`

export const ConfigTitleRowTitle = styled(ConfigTitle)`
  margin: 0;
  flex-shrink: 0;
`

export const ConfigTitleStatus = styled.span<{ $error?: boolean }>`
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  font-size: 11px;
  color: ${props => props.$error ? palette.red : palette.textMuted};
`

/* For showing, e.g., session state. */
export const StateRow = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  min-height: 18px;
  margin-bottom: 1px;
  margin-right: 0;
  font-size: 12px;
`

export const StateTitle = styled.span`
  font-weight: 600;
  color: ${palette.textSecondary};
  margin-right: 0.6rem;
`

export const IndentedStateTitle = styled(StateTitle)`
  margin-left: 10px;
  font-weight: normal;
  color: ${palette.textMuted};
`

export const DoubleIndentedStateTitle = styled(StateTitle)`
  margin-left: 20px;
  font-weight: normal;
  color: ${palette.textMuted};
`

/* Telemetry values and states. The UI font with tabular numerals keeps changing numbers aligned;
   monospace is reserved for logs, IDs/hashes and code-like content. */
export const StateValue = styled.span`
  font-size: 11.5px;
  font-variant-numeric: tabular-nums;
  color: ${palette.text};
`

/* If enabled, grays out all elements inside the panel. */
export const GrayedOutPanel = styled.div<{ isGrayedOut: boolean }>`
  filter: ${(props) => (props.isGrayedOut ? 'grayscale(100%)' : 'none')};
  opacity: ${(props) => (props.isGrayedOut ? '0.3' : '1')};
  transition: filter 0.3s ease, opacity 0.3s ease;
`
