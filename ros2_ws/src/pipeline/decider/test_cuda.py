import torch

bold_on = "\033[1m"
bold_off = "\033[0m"

def test_cuda():
    if torch.cuda.is_available():
        print(f"CUDA is {bold_on}available{bold_off}.")
        print(f"GPU: {bold_on}{torch.cuda.get_device_name(torch.cuda.current_device())}{bold_off}")
        return True
    else:
        print("CUDA is {bold_on}not available{bold_off}.")
        return False

if __name__ == "__main__":
    success = test_cuda()
    exit(0 if success else 1)
