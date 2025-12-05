const pxToRem = (px: number) => `${px / 16}rem`

const theme = {
  rem: pxToRem,
  colors: {
    red: 'red',
    primary: 'black',
    disabled: 'grey',
    error: '#a21010',
    ok: '#258f25',
    white: '#FFFFFF',
    lightergray: '#f1f1f1',
    lightgray: '#e0e0e0',
    gray: '#b0b0b0',
    darkgray: '#707070',
    green: '#52d045',
    blue: '#4591d0',
    brown: '#bb611a',
    yellow: '#d9cf2b',
  },
  spacing: {},
  typography: {
    large: pxToRem(15),
    medium: pxToRem(11),
    small: pxToRem(8),
  },
  borderRadius: pxToRem(3),
}

export type Theme = typeof theme

export type Color = keyof Theme['colors']
export type Spacing = keyof Theme['spacing']

export default theme
