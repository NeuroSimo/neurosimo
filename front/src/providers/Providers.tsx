import React from 'react'
import { ThemeProvider } from 'styled-components'

import theme from 'styles/theme'

interface Props {
  children: React.ReactNode
}

const Providers: React.FC<Props> = (props: Props) => <ThemeProvider theme={theme}>{props.children}</ThemeProvider>

export default Providers
