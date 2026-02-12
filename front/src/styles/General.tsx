import styled from 'styled-components'

export const Select = styled.select`
  width: 150px;
  padding: 5px;
  border: 1px solid #ccc;
  border-radius: 3px;
  outline: none;
  transition: background-color 0.2s;
  appearance: none;
  margin-right: 17px;

  &:focus {
    background-color: transparent;
  }
`

// Shared width for configuration panels
export const CONFIG_PANEL_WIDTH = 320

/* Shared dashboard panel title offset (px). */
export const DASHBOARD_PANEL_OFFSET_FROM_TOP = 220
export const DASHBOARD_PANEL_HEIGHT = 300

export const StyledPanel = styled.div<{ isGrayedOut?: boolean }>`
  padding: 8px 0px 12px 21px;
  border-radius: 3px;
  background-color: #f7f7f7;
  box-shadow: 0px 3px 6px rgba(0, 0, 0, 0.1);/
  ${({ isGrayedOut }) =>
    isGrayedOut &&
    `
    filter: grayscale(100%);
    color: #aaa;
    background-color: #f0f0f0;
    transition: filter 0.3s ease, opacity 0.3s ease;
    pointer-events: none;
  `}
`

export interface ActiveProps {
  isActive?: boolean
  isHidden?: boolean
}

export const StyledButton = styled.button<ActiveProps>`
  width: 123px;
  height: 31px;

  font-size: 0.8rem;
  padding: 0.48rem 0.35rem;
  margin-bottom: 0.48rem;
  border: none;
  border-radius: 3px;
  background-color: #007bff;
  color: white;
  cursor: pointer;

  &:hover {
    background-color: #0056b3;
  }
  &:disabled {
    background-color: #cccccc;
    color: #888888;
  }
  &:hover:disabled {
    background-color: #cccccc;
  }
  transition: opacity 0.2s;

  ${(props) =>
    props.isHidden &&
    `
    opacity: 0.0;
  `}
`

export const StyledRedButton = styled(StyledButton)`
  background-color: #a00000;
  &:hover {
    background-color: #700000;
  }
`

export const TabBar = styled.div`
  margin: 0.31rem;

  a {
    text-decoration: none;
    color: #505050;
    padding: 0.31rem;
    display: inline-block;
    transition: color 0.3s ease;

    &:hover {
      color: #303030;
    }

    &.active {
      color: #222222;
      font-weight: bold;
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
  margin-bottom: 5px;
  padding-right: 0px;
`

export const CloseConfigRow = styled(ConfigRow)`
  margin-bottom: 4px;
`

export const ConfigLabel = styled.label`
  width: 185px;
  font-size: 11px;
  font-family: 'Roboto', 'Segoe UI', sans-serif;
  color: #333;
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
  font-size: 11px;
  font-family: 'Roboto', 'Segoe UI', sans-serif;
  color: #333;
  margin-right: 20px;
`

export const NotesValue = styled.div`
  width: 190px;
  text-align: right;
  font-size: 11px;
  font-family: 'Roboto', 'Segoe UI', sans-serif;
  color: #333;
  margin-right: 20px;
  white-space: pre-wrap;
  word-wrap: break-word;
  overflow-wrap: break-word;
  max-height: 55px;
  overflow-y: auto;
`

export const SmallerTitle = styled.h2`
  font-size: 12px;
  text-align: center;
  margin-bottom: 14px;
  margin-right: 18px;
  font-weight: bold;
`

/* For showing, e.g., session state. */
export const StateRow = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.3rem;
  margin-right: 9px;
`

export const StateTitle = styled.span`
  font-weight: bold;
  margin-right: 0.6rem;
`

export const IndentedStateTitle = styled(StateTitle)`
  margin-left: 12px;
  font-weight: normal;
`

export const DoubleIndentedStateTitle = styled(StateTitle)`
  margin-left: 25px;
  font-weight: normal;
`

export const StateValue = styled.span``

/* If enabled, grays out all elements inside the panel. */
export const GrayedOutPanel = styled.div<{ isGrayedOut: boolean }>`
  filter: ${(props) => (props.isGrayedOut ? 'grayscale(100%)' : 'none')};
  opacity: ${(props) => (props.isGrayedOut ? '0.3' : '1')};
  transition: filter 0.3s ease, opacity 0.3s ease;
`
