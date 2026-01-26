export const formatTime = (time: number | undefined): string => {
  if (time == undefined) {
    return ''
  }
  time = Math.round(time)

  const hours = Math.floor(time / 3600)
  const minutes = Math.floor((time % 3600) / 60)
  const seconds = time % 60

  let result = ''

  if (hours > 0) {
    result += `${hours} h `
  }
  if (minutes > 0) {
    result += `${minutes} min `
  }
  if (seconds > 0 || result === '') {
    result += `${seconds} s`
  }

  return result.trim()
}

export function formatFrequency(frequency: number | undefined): string {
  if (frequency === undefined || frequency === 0) {
    return '\u2013'
  }

  if (frequency < 1000) {
    return `${frequency} Hz`
  }

  const frequencyInKHz = frequency / 1000
  return `${frequencyInKHz} kHz`
}

export function formatDateTime(dateTime: string | Date): string {
  const date = new Date(dateTime)
  const day = date.getDate()
  const month = date.getMonth() + 1
  const year = date.getFullYear()
  const hours = date.getHours().toString().padStart(2, '0')
  const minutes = date.getMinutes().toString().padStart(2, '0')
  const seconds = date.getSeconds().toString().padStart(2, '0')
  return `${day}.${month}.${year}, ${hours}:${minutes}:${seconds}`
}
