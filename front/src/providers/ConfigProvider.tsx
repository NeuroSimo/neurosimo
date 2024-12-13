import React, { useEffect, useState, ReactNode } from 'react'

export const TargetingAlgorithms = {
  LEAST_SQUARES: 0,
  GENETIC: 1,
}

interface ConfigContextType {
  targetingAlgorithm: number
  setTargetingAlgorithm: React.Dispatch<React.SetStateAction<number>>
}

const defaultConfigState: ConfigContextType = {
  targetingAlgorithm: TargetingAlgorithms.LEAST_SQUARES,
  setTargetingAlgorithm: () => {
    console.warn('setTargetingAlgorithm is not yet initialized.')
  },
}

export const ConfigContext = React.createContext<ConfigContextType>(defaultConfigState)

interface ConfigProviderProps {
  children: ReactNode
}

/* Session storage utilities. */

const getData = (): any => {
  const data = sessionStorage.getItem('config')
  return data ? JSON.parse(data) : {}
}

const storeKey = (key: string, value: any) => {
  const currentData = getData()
  currentData[key] = value
  sessionStorage.setItem('config', JSON.stringify(currentData))
}

const getKey = (key: string, defaultValue: any): any => {
  const data = getData()
  return key in data ? data[key] : defaultValue
}

export const ConfigProvider: React.FC<ConfigProviderProps> = ({ children }) => {
  const [targetingAlgorithm, setTargetingAlgorithm] = useState<number>(
    getKey('targetingAlgorithm', TargetingAlgorithms.LEAST_SQUARES),
  )

  /* Update session storage. */
  useEffect(() => {
    storeKey('targetingAlgorithm', targetingAlgorithm)
  }, [targetingAlgorithm])

  return (
    <ConfigContext.Provider value={{ targetingAlgorithm, setTargetingAlgorithm }}>{children}</ConfigContext.Provider>
  )
}
